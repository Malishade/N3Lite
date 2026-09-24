using System;
using System.Runtime.InteropServices;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// One cell's worth of static collision geometry: a triangle soup in playfield world coordinates,
    /// from the RDB's surface resources (record type <b>1000013</b>).
    ///
    /// <para>
    /// The triangles sit in a BVH, since a cell can hold hundreds of them. The tree is an acceleration structure only: the nearest hit along a segment is the
    /// same whatever structure finds it. What is <i>not</i> merely an optimisation is
    /// <see cref="CalculateClosestPoint"/> — see the note there.
    /// </para>
    /// </summary>
    public sealed class TriangleMeshSurface : ISurface
    {
        readonly Vec3[] _vertices;
        readonly int[] _indices;

        // A world AABB, for rejecting a segment before touching any triangle.
        readonly float _minX, _minY, _minZ, _maxX, _maxY, _maxZ;

        // ---- BVH over the triangles (Bikker, "How to build a BVH", parts 1-3) ----
        //
        // Nodes are flat arrays: six padded bounds per node, and leftFirst/count packed as in the
        // 32-byte node — count == 0 means an interior node whose children are leftFirst and
        // leftFirst + 1; count > 0 means a leaf over _bvhTris[leftFirst .. leftFirst + count).
        // _bvhTris holds each triangle's three vertices in leaf order, _bvhTriId its original index,
        // which breaks ties so the answer is the one the in-order sweep would have given.

        /// <summary>
        /// Slack on every node box. <see cref="TilemapSurface.EdgeEpsilon"/> is negative, so
        /// <c>RayTriangle</c> accepts hits a little outside the triangle; the box must not cull them.
        /// </summary>
        internal const float BoundsPad = 0.05f;

        const int LeafSize = 4;
        const int SahBins = 12;

        /// <summary>The BVH's depth limit, which also sizes both traversal stacks.</summary>
        internal const int MaxDepth = 64;

        float[] _nodeBounds;
        int[] _nodeLeftFirst;
        int[] _nodeCount;
        int _nodesUsed;
        Vec3[] _bvhTris;
        int[] _bvhTriId;

        // Per triangle in leaf order, nine floats: e0, e1 and n = Cross(e0, e1) as RayTriangle builds
        // them for the swapped winding (v0, v2, v1) -- e0 = v2 - v0, e1 = v1 - v0.
        float[] _triPlane;

        // The six arrays above, pinned for the life of the surface, and their addresses. A query hands
        // the kernels these stored pointers instead of pinning six arrays on every call. The finalizer
        // frees the pins when the surface is dropped.
        GCHandle[] _pins;
        TriangleMeshKernels.BvhView _bvh;

        /// <summary>
        /// <paramref name="indices"/> is triples into <paramref name="vertices"/>. Both are taken by
        /// reference and must not be mutated afterwards.
        /// </summary>
        public TriangleMeshSurface(Vec3[] vertices, int[] indices)
        {
            _vertices = vertices ?? Array.Empty<Vec3>();
            _indices = indices ?? Array.Empty<int>();

            _minX = _minY = _minZ = float.MaxValue;
            _maxX = _maxY = _maxZ = float.MinValue;
            foreach (Vec3 v in _vertices)
            {
                if (v.X < _minX) _minX = v.X;
                if (v.X > _maxX) _maxX = v.X;
                if (v.Y < _minY) _minY = v.Y;
                if (v.Y > _maxY) _maxY = v.Y;
                if (v.Z < _minZ) _minZ = v.Z;
                if (v.Z > _maxZ) _maxZ = v.Z;
            }

            BuildBvh();
        }

        public int TriangleCount => _indices.Length / 3;

        /// <summary>The world AABB of the geometry, or an inverted box when empty.</summary>
        public void GetBounds(out Vec3 min, out Vec3 max)
        {
            min = new Vec3(_minX, _minY, _minZ);
            max = new Vec3(_maxX, _maxY, _maxZ);
        }

        // ---- line query -----------------------------------------------------

        /// <summary>
        /// The nearest triangle hit along the segment, tested with the same numerics as the terrain
        /// (<see cref="TilemapSurface.RayTriangle"/>), so the normal handed back already faces the
        /// caller. It is normalised here, as the terrain's is.
        ///
        /// <para>
        /// <b>One-sided, with the record's winding reversed.</b> A character placed inside static
        /// geometry can walk back out of it, which a two-sided test would not allow. With the record's
        /// own order (v0, v1, v2) walls were solid from the inside, so the test uses (v0, v2, v1).
        /// </para>
        /// </summary>
        public bool GetLineIntersection(
            Vec3 start, Vec3 end, out Vec3 hit, out Vec3 normal, bool clipToBounds, object locality)
        {
            hit = Vec3.Zero;
            normal = Vec3.ReferenceUp;

            if (_nodeCount == null || !SegmentHitsBounds(start, end))
                return false;

            // Locals, not fields: the kernel is handed stack addresses only, never the inside of an
            // object the collector could move.
            TriangleMeshKernels.BvhView bvh = _bvh;
            bool found = TriangleMeshKernels.LineIntersection(in bvh, in start, in end, out Vec3 h, out Vec3 n) != 0;
            hit = h;
            normal = n;
            return found;
        }

        // ---- BVH build: binned SAH over triangle centroids ----------------

        void BuildBvh()
        {
            int n = _indices.Length / 3;
            if (n == 0)
                return;

            _bvhTris = new Vec3[n * 3];
            _bvhTriId = new int[n];
            var centroids = new Vec3[n];
            for (int t = 0; t < n; t++)
            {
                Vec3 v0 = _vertices[_indices[t * 3]];
                Vec3 v1 = _vertices[_indices[t * 3 + 1]];
                Vec3 v2 = _vertices[_indices[t * 3 + 2]];
                _bvhTris[t * 3] = v0;
                _bvhTris[t * 3 + 1] = v1;
                _bvhTris[t * 3 + 2] = v2;
                _bvhTriId[t] = t;
                centroids[t] = (v0 + v1 + v2) * (1f / 3f);
            }

            int capacity = 2 * n - 1;
            _nodeBounds = new float[capacity * 6];
            _nodeLeftFirst = new int[capacity];
            _nodeCount = new int[capacity];

            _nodeLeftFirst[0] = 0;
            _nodeCount[0] = n;
            _nodesUsed = 1;
            Subdivide(0, 1, centroids);

            // After the partition, so the planes follow the leaf order.
            _triPlane = new float[n * 9];
            for (int k = 0; k < n; k++)
            {
                Vec3 v0 = _bvhTris[k * 3];
                Vec3 v1 = _bvhTris[k * 3 + 1];
                Vec3 v2 = _bvhTris[k * 3 + 2];
                float e0x = v2.X - v0.X, e0y = v2.Y - v0.Y, e0z = v2.Z - v0.Z;
                float e1x = v1.X - v0.X, e1y = v1.Y - v0.Y, e1z = v1.Z - v0.Z;
                int p = k * 9;
                _triPlane[p] = e0x;
                _triPlane[p + 1] = e0y;
                _triPlane[p + 2] = e0z;
                _triPlane[p + 3] = e1x;
                _triPlane[p + 4] = e1y;
                _triPlane[p + 5] = e1z;
                _triPlane[p + 6] = e0y * e1z - e0z * e1y;
                _triPlane[p + 7] = e0z * e1x - e0x * e1z;
                _triPlane[p + 8] = e0x * e1y - e0y * e1x;
            }

            PinBvh();
        }

        unsafe void PinBvh()
        {
            _pins = new[]
            {
                GCHandle.Alloc(_nodeBounds, GCHandleType.Pinned),
                GCHandle.Alloc(_nodeLeftFirst, GCHandleType.Pinned),
                GCHandle.Alloc(_nodeCount, GCHandleType.Pinned),
                GCHandle.Alloc(_bvhTris, GCHandleType.Pinned),
                GCHandle.Alloc(_bvhTriId, GCHandleType.Pinned),
                GCHandle.Alloc(_triPlane, GCHandleType.Pinned),
            };

            _bvh = new TriangleMeshKernels.BvhView
            {
                NodeBounds = (float*)_pins[0].AddrOfPinnedObject(),
                NodeLeftFirst = (int*)_pins[1].AddrOfPinnedObject(),
                NodeCount = (int*)_pins[2].AddrOfPinnedObject(),
                Tris = (Vec3*)_pins[3].AddrOfPinnedObject(),
                TriId = (int*)_pins[4].AddrOfPinnedObject(),
                TriPlane = (float*)_pins[5].AddrOfPinnedObject(),
            };
        }

        ~TriangleMeshSurface()
        {
            if (_pins == null)
                return;

            for (int i = 0; i < _pins.Length; i++)
                if (_pins[i].IsAllocated)
                    _pins[i].Free();
        }

        void Subdivide(int node, int depth, Vec3[] centroids)
        {
            int first = _nodeLeftFirst[node];
            int count = _nodeCount[node];

            TriangleBounds(first, count, out Vec3 min, out Vec3 max);
            StoreBounds(node, min, max);

            // The traversal stack holds at most one node per level.
            if (count <= LeafSize || depth >= MaxDepth)
                return;

            if (!FindSplit(first, count, centroids, out int axis, out float split, out float cost))
                return;

            // Bikker part 2: split only when it beats leaving the triangles in this node.
            if (cost >= count * HalfArea(min, max))
                return;

            // In-place partition on the centroid, carrying vertices, ids and centroids together.
            int i = first;
            int j = first + count - 1;
            while (i <= j)
            {
                if (Axis(centroids[i], axis) < split)
                {
                    i++;
                    continue;
                }

                SwapTriangle(i, j, centroids);
                j--;
            }

            int leftCount = i - first;
            if (leftCount == 0 || leftCount == count)
                return;

            int left = _nodesUsed++;
            int right = _nodesUsed++;
            _nodeLeftFirst[left] = first;
            _nodeCount[left] = leftCount;
            _nodeLeftFirst[right] = i;
            _nodeCount[right] = count - leftCount;

            _nodeLeftFirst[node] = left;
            _nodeCount[node] = 0;

            Subdivide(left, depth + 1, centroids);
            Subdivide(right, depth + 1, centroids);
        }

        /// <summary>
        /// Binned SAH (Bikker part 3): the cheapest of <see cref="SahBins"/> - 1 planes per axis over
        /// the centroid bounds, costed as leftCount * leftArea + rightCount * rightArea.
        /// </summary>
        bool FindSplit(int first, int count, Vec3[] centroids, out int bestAxis, out float bestSplit, out float bestCost)
        {
            bestAxis = -1;
            bestSplit = 0f;
            bestCost = float.MaxValue;

            var binMin = new Vec3[SahBins];
            var binMax = new Vec3[SahBins];
            var binCount = new int[SahBins];
            var leftArea = new float[SahBins - 1];
            var leftCount = new int[SahBins - 1];

            for (int axis = 0; axis < 3; axis++)
            {
                float cMin = float.MaxValue, cMax = float.MinValue;
                for (int k = first; k < first + count; k++)
                {
                    float c = Axis(centroids[k], axis);
                    if (c < cMin) cMin = c;
                    if (c > cMax) cMax = c;
                }

                if (cMax <= cMin)
                    continue;

                for (int b = 0; b < SahBins; b++)
                {
                    binMin[b] = new Vec3(float.MaxValue, float.MaxValue, float.MaxValue);
                    binMax[b] = new Vec3(float.MinValue, float.MinValue, float.MinValue);
                    binCount[b] = 0;
                }

                float scale = SahBins / (cMax - cMin);
                for (int k = first; k < first + count; k++)
                {
                    int b = Math.Min(SahBins - 1, (int)((Axis(centroids[k], axis) - cMin) * scale));
                    binCount[b]++;
                    for (int v = 0; v < 3; v++)
                        Grow(ref binMin[b], ref binMax[b], _bvhTris[k * 3 + v]);
                }

                int lCount = 0;
                Vec3 accMin = new Vec3(float.MaxValue, float.MaxValue, float.MaxValue);
                Vec3 accMax = new Vec3(float.MinValue, float.MinValue, float.MinValue);
                for (int b = 0; b < SahBins - 1; b++)
                {
                    lCount += binCount[b];
                    if (binCount[b] > 0)
                    {
                        Grow(ref accMin, ref accMax, binMin[b]);
                        Grow(ref accMin, ref accMax, binMax[b]);
                    }
                    leftCount[b] = lCount;
                    leftArea[b] = lCount > 0 ? HalfArea(accMin, accMax) : 0f;
                }

                int rCount = 0;
                accMin = new Vec3(float.MaxValue, float.MaxValue, float.MaxValue);
                accMax = new Vec3(float.MinValue, float.MinValue, float.MinValue);
                for (int b = SahBins - 1; b > 0; b--)
                {
                    rCount += binCount[b];
                    if (binCount[b] > 0)
                    {
                        Grow(ref accMin, ref accMax, binMin[b]);
                        Grow(ref accMin, ref accMax, binMax[b]);
                    }

                    int l = leftCount[b - 1];
                    if (l == 0 || rCount == 0)
                        continue;

                    float cost = l * leftArea[b - 1] + rCount * HalfArea(accMin, accMax);
                    if (cost < bestCost)
                    {
                        bestCost = cost;
                        bestAxis = axis;
                        bestSplit = cMin + b / scale;
                    }
                }
            }

            return bestAxis >= 0;
        }

        void TriangleBounds(int first, int count, out Vec3 min, out Vec3 max)
        {
            min = new Vec3(float.MaxValue, float.MaxValue, float.MaxValue);
            max = new Vec3(float.MinValue, float.MinValue, float.MinValue);
            for (int k = first * 3; k < (first + count) * 3; k++)
                Grow(ref min, ref max, _bvhTris[k]);
        }

        void StoreBounds(int node, Vec3 min, Vec3 max)
        {
            int i = node * 6;
            _nodeBounds[i] = min.X - BoundsPad;
            _nodeBounds[i + 1] = min.Y - BoundsPad;
            _nodeBounds[i + 2] = min.Z - BoundsPad;
            _nodeBounds[i + 3] = max.X + BoundsPad;
            _nodeBounds[i + 4] = max.Y + BoundsPad;
            _nodeBounds[i + 5] = max.Z + BoundsPad;
        }

        void SwapTriangle(int a, int b, Vec3[] centroids)
        {
            for (int v = 0; v < 3; v++)
                (_bvhTris[a * 3 + v], _bvhTris[b * 3 + v]) = (_bvhTris[b * 3 + v], _bvhTris[a * 3 + v]);
            (_bvhTriId[a], _bvhTriId[b]) = (_bvhTriId[b], _bvhTriId[a]);
            (centroids[a], centroids[b]) = (centroids[b], centroids[a]);
        }

        static void Grow(ref Vec3 min, ref Vec3 max, Vec3 p)
        {
            if (p.X < min.X) min.X = p.X;
            if (p.Y < min.Y) min.Y = p.Y;
            if (p.Z < min.Z) min.Z = p.Z;
            if (p.X > max.X) max.X = p.X;
            if (p.Y > max.Y) max.Y = p.Y;
            if (p.Z > max.Z) max.Z = p.Z;
        }

        static float HalfArea(Vec3 min, Vec3 max)
        {
            float x = max.X - min.X, y = max.Y - min.Y, z = max.Z - min.Z;
            return x * y + y * z + z * x;
        }

        static float Axis(Vec3 v, int axis) => axis == 0 ? v.X : axis == 1 ? v.Y : v.Z;

        /// <summary>True when <paramref name="p"/> lies within <see cref="BoundsPad"/> of the triangle's box.</summary>
        internal static bool NearTriangle(Vec3 p, Vec3 v0, Vec3 v1, Vec3 v2)
            => TriangleMeshKernels.NearTriangle(p, v0, v1, v2);

        bool SegmentHitsBounds(Vec3 a, Vec3 b)
        {
            if (_minX > _maxX)
                return false;

            if (Math.Max(a.X, b.X) < _minX || Math.Min(a.X, b.X) > _maxX) return false;
            if (Math.Max(a.Y, b.Y) < _minY || Math.Min(a.Y, b.Y) > _maxY) return false;
            if (Math.Max(a.Z, b.Z) < _minZ || Math.Min(a.Z, b.Z) > _maxZ) return false;
            return true;
        }

        // ---- closest point --------------------------------------------------

        /// <summary>
        /// The <b>floor</b> under <paramref name="point"/>: the highest triangle whose XZ projection
        /// contains it and which is not above it. Returns
        /// <see cref="TilemapSurface.NoClosestPoint"/> when nothing is under it, which makes the caller
        /// keep the terrain.
        ///
        /// <para>
        /// <b>A floor, not a 3D closest point.</b> A true closest point would be wrong here: the ground
        /// clamp raises the body to <c>closest.Y + stepHeight</c>, so standing beside a wall would
        /// return a point on the wall at roughly the body's own height and lift the body a step every
        /// frame — it would climb walls. A downward floor query is the only reading that makes the clamp behave, and it
        /// is what lets a body stand on a statel deck while still walking under a bridge.
        /// </para>
        /// </summary>
        public void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality)
        {
            // TilemapSurface.NoClosestPoint, not the point itself: a surface that returns the point it
            // was handed beats the terrain from any height and the ground clamp stops working.
            closest = new Vec3(point.X, TilemapSurface.NoClosestPoint, point.Z);
            normal = Vec3.ReferenceUp;

            if (_nodeCount == null)
                return;
            if (point.X < _minX || point.X > _maxX || point.Z < _minZ || point.Z > _maxZ)
                return;

            TriangleMeshKernels.BvhView bvh = _bvh;
            if (TriangleMeshKernels.Floor(in bvh, in point, out float floorY, out Vec3 floorNormal) == 0)
                return;

            closest = new Vec3(point.X, floorY, point.Z);
            normal = floorNormal;
        }

        /// <summary>Never vetoes, like every other surface.</summary>
        public bool VetoPosition(ref Vec3 position, object locality, Vec3 previous) => false;
    }
}

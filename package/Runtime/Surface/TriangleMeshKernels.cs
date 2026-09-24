using System;
using System.Runtime.CompilerServices;
using Unity.Burst;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// The two <see cref="TriangleMeshSurface"/> queries every body runs every frame — the line and the
    /// floor — written over raw pointers so Burst can compile them. Outside Unity, or with Burst off,
    /// they run as ordinary C#.
    ///
    /// <para>
    /// The surface pins its own arrays and passes them in as a <see cref="BvhView"/>, so nothing is
    /// copied and the geometry's lifetime is unchanged.
    /// </para>
    ///
    /// <para>
    /// <b>Strict float mode:</b> every operation stays in source order in single precision, with no
    /// fused multiply-adds, which is what .NET computes too. A runtime that evaluates float
    /// expressions in double precision (Unity's Mono does) differs in the last bits; measured over
    /// 200,000 rays, never in a hit/miss or in which triangle wins, and by at most 61 µm on a hit.
    /// </para>
    /// </summary>
    [BurstCompile(FloatMode = FloatMode.Strict, CompileSynchronously = true)]
    public static unsafe class TriangleMeshKernels
    {
        /// <summary>A <see cref="TriangleMeshSurface"/>'s BVH, pinned for the length of one query.</summary>
        public struct BvhView
        {
            /// <summary>Six padded bounds per node: min x, y, z, then max x, y, z.</summary>
            public float* NodeBounds;
            public int* NodeLeftFirst;
            /// <summary>0 for an interior node (children leftFirst, leftFirst + 1), else a leaf's triangle count.</summary>
            public int* NodeCount;
            /// <summary>Three vertices per triangle, in leaf order.</summary>
            public Vec3* Tris;
            /// <summary>Each triangle's original index, for the tie-break.</summary>
            public int* TriId;
            /// <summary>Nine floats per triangle: e0, e1, n for the swapped winding.</summary>
            public float* TriPlane;
        }

        // ---- line query -----------------------------------------------------

        /// <summary>
        /// The body of <see cref="TriangleMeshSurface.GetLineIntersection"/> after its bounds reject:
        /// the nearest hit along the segment, 1 when there is one.
        /// </summary>
        [BurstCompile(FloatMode = FloatMode.Strict, CompileSynchronously = true)]
        public static int LineIntersection(in BvhView bvh, in Vec3 start, in Vec3 end, out Vec3 hit, out Vec3 normal)
        {
            hit = new Vec3(0f, 0f, 0f);
            normal = new Vec3(0f, 1f, 0f);

            Vec3 origin = start;
            Vec3 direction = end - origin;
            float length = direction.Length;
            if (length <= 0f)
                return 0;

            direction = direction * (1f / length);

            // Per-axis reciprocals for the slab test. An exactly-zero component (every vertical probe)
            // is handled as a containment test instead, so no 0 * inf NaN reaches the comparisons.
            bool zeroX = direction.X == 0f, zeroY = direction.Y == 0f, zeroZ = direction.Z == 0f;
            float invX = zeroX ? 0f : 1f / direction.X;
            float invY = zeroY ? 0f : 1f / direction.Y;
            float invZ = zeroZ ? 0f : 1f / direction.Z;

            bool found = false;
            float best = 0f;
            int bestId = 0;

            // Nearest-child-first traversal with an explicit stack. The far child is re-tested on pop
            // against the shortened segment, so a hit in the near child culls it.
            int* stack = stackalloc int[TriangleMeshSurface.MaxDepth];
            int sp = 0;
            int node = 0;

            if (!Slab(bvh.NodeBounds, node, origin, invX, invY, invZ, zeroX, zeroY, zeroZ, length, out _))
                return 0;

            while (true)
            {
                int count = bvh.NodeCount[node];
                if (count > 0)
                {
                    int first = bvh.NodeLeftFirst[node];
                    for (int k = first; k < first + count; k++)
                    {
                        // TilemapSurface.RayTriangle(start, direction, v0, v2, v1) -- v2/v1 swapped,
                        // the records wind opposite to terrain tiles -- with the edges and the plane
                        // normal taken from TriPlane, which holds exactly what it would compute.
                        float* plane = bvh.TriPlane + k * 9;
                        float nx = plane[6], ny = plane[7], nz = plane[8];
                        float denominator = direction.X * nx + direction.Y * ny + direction.Z * nz;
                        if (denominator <= 0f)
                            continue;

                        Vec3 v0 = bvh.Tris[k * 3];
                        float tRay = ((v0.X - origin.X) * nx + (v0.Y - origin.Y) * ny + (v0.Z - origin.Z) * nz) / denominator;
                        float hx = origin.X + direction.X * tRay;
                        float hy = origin.Y + direction.Y * tRay;
                        float hz = origin.Z + direction.Z * tRay;

                        Vec3 v1 = bvh.Tris[k * 3 + 1];
                        Vec3 v2 = bvh.Tris[k * 3 + 2];
                        Vec3 n = default;
                        if (!TilemapSurface.EdgeTests(hx, hy, hz, v0, v2, v1,
                                plane[0], plane[1], plane[2], plane[3], plane[4], plane[5],
                                nx, ny, nz, ref n))
                            continue;

                        Vec3 h = new Vec3(hx, hy, hz);

                        // EdgeEpsilon is an absolute slack sized for 4 m terrain tiles. On a sliver
                        // triangle it widens into a strip of the plane metres long, and a ray through
                        // open air "hits" it -- 60 m out was measured on random geometry. Only hits
                        // within TriangleMeshSurface.BoundsPad of the triangle itself count.
                        if (!NearTriangle(h, v0, v1, v2))
                            continue;

                        float t = Vec3.Dot(h - origin, direction);
                        if (t < 0f || t > length)               // behind the start, or past the end
                            continue;

                        // Nearer wins; an exact tie goes to the lower original index, which is the
                        // triangle an in-order sweep would have kept.
                        int id = bvh.TriId[k];
                        if (found && (t > best || (t == best && id > bestId)))
                            continue;

                        float nl = n.Length;                    // RayTriangle does not normalise
                        hit = h;
                        normal = nl > 0f ? n * (1f / nl) : new Vec3(0f, 1f, 0f);
                        best = t;
                        bestId = id;
                        found = true;
                    }
                }
                else
                {
                    float limit = found ? best : length;
                    int a = bvh.NodeLeftFirst[node];
                    int b = a + 1;
                    bool hitA = Slab(bvh.NodeBounds, a, origin, invX, invY, invZ, zeroX, zeroY, zeroZ, limit, out float ta);
                    bool hitB = Slab(bvh.NodeBounds, b, origin, invX, invY, invZ, zeroX, zeroY, zeroZ, limit, out float tb);

                    if (hitA && hitB)
                    {
                        if (tb < ta)
                        {
                            int swap = a;
                            a = b;
                            b = swap;
                        }
                        stack[sp++] = b;
                        node = a;
                        continue;
                    }

                    if (hitA) { node = a; continue; }
                    if (hitB) { node = b; continue; }
                }

                // Pop the next deferred node that still overlaps what is left of the segment.
                bool next = false;
                while (sp > 0)
                {
                    node = stack[--sp];
                    if (Slab(bvh.NodeBounds, node, origin, invX, invY, invZ, zeroX, zeroY, zeroZ, found ? best : length, out _))
                    {
                        next = true;
                        break;
                    }
                }

                if (!next)
                    break;
            }

            return found ? 1 : 0;
        }

        /// <summary>
        /// Ray/box slab test against node <paramref name="node"/>, over the ray parameter range
        /// [0, <paramref name="limit"/>] (inclusive, so a tie at the current best is still visited).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool Slab(float* bounds, int node, Vec3 o, float invX, float invY, float invZ,
            bool zeroX, bool zeroY, bool zeroZ, float limit, out float tEnter)
        {
            int i = node * 6;
            float tMin = float.NegativeInfinity;
            float tMax = float.PositiveInfinity;
            tEnter = 0f;

            float lo = bounds[i], hi = bounds[i + 3];
            if (zeroX)
            {
                if (o.X < lo || o.X > hi) return false;
            }
            else
            {
                float t1 = (lo - o.X) * invX, t2 = (hi - o.X) * invX;
                if (t1 > t2) { float s = t1; t1 = t2; t2 = s; }
                if (t1 > tMin) tMin = t1;
                if (t2 < tMax) tMax = t2;
            }

            lo = bounds[i + 1]; hi = bounds[i + 4];
            if (zeroY)
            {
                if (o.Y < lo || o.Y > hi) return false;
            }
            else
            {
                float t1 = (lo - o.Y) * invY, t2 = (hi - o.Y) * invY;
                if (t1 > t2) { float s = t1; t1 = t2; t2 = s; }
                if (t1 > tMin) tMin = t1;
                if (t2 < tMax) tMax = t2;
            }

            lo = bounds[i + 2]; hi = bounds[i + 5];
            if (zeroZ)
            {
                if (o.Z < lo || o.Z > hi) return false;
            }
            else
            {
                float t1 = (lo - o.Z) * invZ, t2 = (hi - o.Z) * invZ;
                if (t1 > t2) { float s = t1; t1 = t2; t2 = s; }
                if (t1 > tMin) tMin = t1;
                if (t2 < tMax) tMax = t2;
            }

            if (tMax < tMin || tMax < 0f || tMin > limit)
                return false;

            tEnter = tMin;
            return true;
        }

        /// <summary>True when <paramref name="p"/> lies within <see cref="TriangleMeshSurface.BoundsPad"/> of the triangle's box.</summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool NearTriangle(Vec3 p, Vec3 v0, Vec3 v1, Vec3 v2)
        {
            if (p.X < Math.Min(v0.X, Math.Min(v1.X, v2.X)) - TriangleMeshSurface.BoundsPad) return false;
            if (p.X > Math.Max(v0.X, Math.Max(v1.X, v2.X)) + TriangleMeshSurface.BoundsPad) return false;
            if (p.Y < Math.Min(v0.Y, Math.Min(v1.Y, v2.Y)) - TriangleMeshSurface.BoundsPad) return false;
            if (p.Y > Math.Max(v0.Y, Math.Max(v1.Y, v2.Y)) + TriangleMeshSurface.BoundsPad) return false;
            if (p.Z < Math.Min(v0.Z, Math.Min(v1.Z, v2.Z)) - TriangleMeshSurface.BoundsPad) return false;
            if (p.Z > Math.Max(v0.Z, Math.Max(v1.Z, v2.Z)) + TriangleMeshSurface.BoundsPad) return false;
            return true;
        }

        // ---- closest point --------------------------------------------------

        /// <summary>
        /// The walk inside <see cref="TriangleMeshSurface.CalculateClosestPoint"/>: the highest
        /// triangle under <paramref name="point"/> that is not above it, 1 when there is one.
        /// </summary>
        [BurstCompile(FloatMode = FloatMode.Strict, CompileSynchronously = true)]
        public static int Floor(in BvhView bvh, in Vec3 point, out float floorY, out Vec3 floorNormal)
        {
            // A float-equality guard, not a game constant: a body resting on a deck sits at the deck's
            // own height, so the surface it stands on must not be excluded by rounding.
            const float Tolerance = 1e-3f;
            float px = point.X, pz = point.Z;
            float ceiling = point.Y + Tolerance;

            bool found = false;
            float bestY = 0f;
            int bestId = 0;
            Vec3 bestNormal = new Vec3(0f, 1f, 0f);

            // The same BVH as the line query, walked as a vertical line: a node is visited only when
            // its XZ box holds the point, its bottom is not above the ceiling, and its top could still
            // beat the best floor so far. Each level pushes two and pops one, so the stack needs at
            // most one slot per level plus the root.
            int* stack = stackalloc int[TriangleMeshSurface.MaxDepth + 2];
            int sp = 0;
            stack[sp++] = 0;

            float* bounds = bvh.NodeBounds;
            while (sp > 0)
            {
                int node = stack[--sp];
                int b = node * 6;
                if (px < bounds[b] || px > bounds[b + 3]
                    || pz < bounds[b + 2] || pz > bounds[b + 5])
                    continue;
                if (bounds[b + 1] > ceiling)
                    continue;
                if (found && bounds[b + 4] < bestY)
                    continue;

                int count = bvh.NodeCount[node];
                if (count == 0)
                {
                    // Pop the child with the higher top first; its floor is the likelier winner.
                    int left = bvh.NodeLeftFirst[node];
                    int right = left + 1;
                    if (bounds[left * 6 + 4] > bounds[right * 6 + 4])
                    {
                        int swap = left;
                        left = right;
                        right = swap;
                    }
                    stack[sp++] = left;
                    stack[sp++] = right;
                    continue;
                }

                int first = bvh.NodeLeftFirst[node];
                for (int k = first; k < first + count; k++)
                {
                    if (!HeightAt(px, pz, bvh.Tris[k * 3], bvh.Tris[k * 3 + 1], bvh.Tris[k * 3 + 2],
                            out float y, out Vec3 n))
                        continue;
                    if (y > ceiling)
                        continue;

                    // Higher wins; an exact tie goes to the lower original index, as the in-order
                    // sweep this replaced would have kept.
                    int id = bvh.TriId[k];
                    if (found && (y < bestY || (y == bestY && id > bestId)))
                        continue;

                    bestY = y;
                    bestId = id;
                    bestNormal = n;
                    found = true;
                }
            }

            floorY = bestY;
            floorNormal = bestNormal;
            return found ? 1 : 0;
        }

        /// <summary>
        /// The triangle's height at an XZ position, or false when the position is outside it or the
        /// triangle is vertical (no height to give). Barycentric, so it matches the plane exactly at
        /// the vertices.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static bool HeightAt(float x, float z, Vec3 v0, Vec3 v1, Vec3 v2, out float y, out Vec3 normal)
        {
            y = 0f;
            normal = new Vec3(0f, 1f, 0f);

            float d = (v1.Z - v2.Z) * (v0.X - v2.X) + (v2.X - v1.X) * (v0.Z - v2.Z);
            if (d == 0f)
                return false;

            float a = ((v1.Z - v2.Z) * (x - v2.X) + (v2.X - v1.X) * (z - v2.Z)) / d;
            if (a < 0f || a > 1f)
                return false;

            float b = ((v2.Z - v0.Z) * (x - v2.X) + (v0.X - v2.X) * (z - v2.Z)) / d;
            if (b < 0f || a + b > 1f)
                return false;

            float c = 1f - a - b;
            if (c < 0f)
                return false;

            y = a * v0.Y + b * v1.Y + c * v2.Y;

            Vec3 n = Vec3.Cross(v1 - v0, v2 - v0);
            if (n.Y < 0f)
                n = -n;                                          // a floor normal always points up
            float length = n.Length;
            normal = length > 0f ? n * (1f / length) : new Vec3(0f, 1f, 0f);
            return true;
        }
    }
}

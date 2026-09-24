using System;
using System.Runtime.CompilerServices;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// The outdoor heightmap as an <see cref="ISurface"/>, with the
    /// statel geometry hanging off it as <see cref="Child"/>.
    ///
    /// <para>
    /// <b>Not implemented:</b>
    /// </para>
    /// <list type="bullet">
    ///   <item>Clipping the ray to the map bounds before walking. A vehicle's own rays are short and
    ///     start inside the map, so this changes nothing for them; a ray from outside the map is not
    ///     clipped, which shifts the segment-containment test at the end.</item>
    ///   <item>Indoor tiles. <see cref="IntersectTile"/> takes the outdoor branch only.</item>
    ///   <item>The veto — see <see cref="VetoPosition"/>.</item>
    /// </list>
    /// </summary>
    public sealed class TilemapSurface : ISurface
    {
        /// <summary>
        /// The edge tolerance in the ray/triangle test. The two edge dot products are compared against
        /// this rather than zero, so a ray passing exactly along a shared edge is accepted by both
        /// triangles instead of falling through the gap.
        /// </summary>
        public const float EdgeEpsilon = -0.0005f;

        /// <summary>The scale in the tile walk's <c>tDelta</c>. It cancels against <c>tMax</c>.</summary>
        const float DdaScale = 100f;

        /// <summary>
        /// The "no closest point" sentinel, <b>-9999</b>. It is written into the candidate's Y
        /// <i>before</i> the child surface is asked, so a child that has nothing to say leaves a value
        /// that can never win the height comparison. A child with no answer must return this rather
        /// than the point it was given — returning the point makes it beat the terrain from any
        /// height, which silently stops the ground clamp pulling a body down.
        /// </summary>
        public const float NoClosestPoint = -9999f;

        readonly ITileHeightSource _tiles;

        public TilemapSurface(ITileHeightSource tiles)
        {
            _tiles = tiles ?? throw new ArgumentNullException(nameof(tiles));
        }

        /// <summary>
        /// A surface tested against the same ray <b>before</b> the terrain, with the nearer of the two
        /// hits returned. Rooms, static meshes and houses compose here (a <see cref="CellSurface"/>)
        /// rather than being walked separately.
        /// </summary>
        public ISurface Child { get; set; }

        // ---- line query: the tile walk ---------------------------------------

        /// <summary>
        /// An Amanatides–Woo walk across the tile grid, bounded by the Manhattan tile distance.
        /// </summary>
        public bool GetLineIntersection(
            Vec3 start,
            Vec3 end,
            out Vec3 hit,
            out Vec3 normal,
            bool clipToBounds,
            object locality)
        {
            hit = Vec3.Zero;
            normal = Vec3.Zero;

            // The nested surface goes first, against the same ray.
            bool hitChild = false;
            Vec3 childHit = Vec3.Zero;
            Vec3 childNormal = Vec3.Zero;
            if (Child != null)
                hitChild = Child.GetLineIntersection(start, end, out childHit, out childNormal, clipToBounds, locality);

            float tileSize = _tiles.TileSize;
            if (tileSize <= 0f)
                return FinishMiss(hitChild, childHit, childNormal, out hit, out normal);

            int width = _tiles.Width;
            int height = _tiles.Height;

            // Tile space.
            float x0 = start.X / tileSize;
            float z0 = start.Z / tileSize;
            float x1 = end.X / tileSize;
            float z1 = end.Z / tileSize;

            int ix = (int)Math.Floor(x0);
            int iz = (int)Math.Floor(z0);
            int ixEnd = (int)Math.Floor(x1);
            int izEnd = (int)Math.Floor(z1);

            SetupAxis(x0, x1, out float tMaxX, out float tDeltaX, out int stepX);
            SetupAxis(z0, z1, out float tMaxZ, out float tDeltaZ, out int stepZ);

            float segmentLengthSquared = (end - start).LengthSquared;

            // n = |dx| + |dz|, and the loop runs while n >= 0
            int n = Math.Abs(ixEnd - ix) + Math.Abs(izEnd - iz);

            while (n >= 0)
            {
                if (ix >= 0 && ix < width && iz >= 0 && iz < height
                    && IntersectTile(ix, iz, start, end, out Vec3 tileHit, out Vec3 tileNormal))
                {
                    // The hit counts only if it lies within the segment, measured as squared distances
                    // from BOTH ends against the segment's own squared length.
                    if ((tileHit - start).LengthSquared <= segmentLengthSquared
                        && (tileHit - end).LengthSquared <= segmentLengthSquared)
                    {
                        if (!hitChild)
                        {
                            hit = tileHit;
                            normal = tileNormal;
                            return true;
                        }

                        // Both hit: the nearer to the ray's start wins, and a tie goes to the TILE
                        // (it is kept whenever tileDistance <= childDistance).
                        if ((tileHit - start).Length > (childHit - start).Length)
                        {
                            hit = childHit;
                            normal = childNormal;
                        }
                        else
                        {
                            hit = tileHit;
                            normal = tileNormal;
                        }

                        return true;
                    }
                }

                if (tMaxZ <= tMaxX)
                {
                    iz += stepZ;
                    tMaxZ += tDeltaZ;
                }
                else
                {
                    ix += stepX;
                    tMaxX += tDeltaX;
                }

                n--;
            }

            return FinishMiss(hitChild, childHit, childNormal, out hit, out normal);
        }

        /// <summary>
        /// Writes <c>(-1, -1, -1)</c> into the out parameter when nothing is hit, so callers that read
        /// it after a false return see that sentinel rather than stale data.
        /// </summary>
        static bool FinishMiss(bool hitChild, Vec3 childHit, Vec3 childNormal, out Vec3 hit, out Vec3 normal)
        {
            if (hitChild)
            {
                hit = childHit;
                normal = childNormal;
                return true;
            }

            hit = new Vec3(-1f, -1f, -1f);
            normal = Vec3.Zero;
            return false;
        }

        /// <summary>
        /// One axis of the walk's setup: <see cref="DdaScale"/> over the tile-space delta, and the
        /// fractional distance to the next tile boundary in the direction of travel.
        ///
        /// <para>
        /// <b>The predicate is <c>delta &lt; 0</c>, not <c>delta &lt;= 0</c>.</b> A delta of exactly
        /// zero takes the positive arm and gets <c>tDelta = 100.0 / 0.0 = +Infinity</c>, so that axis
        /// never steps. With <c>&lt;=</c> it would get <c>-Infinity</c>, which compares as the smaller
        /// <c>tMax</c> forever: the walk would step that axis on every iteration and a ray running
        /// exactly along the other axis would never reach anything
        /// (<c>AShallowRayWalksAcrossTilesUntilItMeetsAWall</c> pins this).
        /// </para>
        /// </summary>
        static void SetupAxis(float from, float to, out float tMax, out float tDelta, out int step)
        {
            float delta = to - from;
            float floor = (float)Math.Floor(from);

            if (delta < 0f)
            {
                tDelta = -DdaScale / delta;      // delta is negative, so tDelta is positive
                step = -1;
                tMax = from - floor;
            }
            else
            {
                tDelta = DdaScale / delta;       // +Infinity when delta is zero
                step = 1;
                tMax = floor + 1f - from;
            }

            tMax *= tDelta;
        }

        // ---- one tile -------------------------------------------------------

        /// <summary>
        /// Two triangles per tile, with a cheap height reject first.
        /// </summary>
        internal bool IntersectTile(int tx, int tz, Vec3 rayStart, Vec3 rayEnd, out Vec3 hit, out Vec3 normal)
        {
            GetTileCorners(tx, tz, out Vec3 c0, out Vec3 c1, out Vec3 c2, out Vec3 c3, out int diagonal);

            // The tile cannot be reached if its highest corner is below both ends of the ray.
            float highest = Math.Max(Math.Max(c0.Y, c1.Y), Math.Max(c2.Y, c3.Y));
            if (highest < rayStart.Y && highest < rayEnd.Y)
            {
                hit = Vec3.Zero;
                normal = Vec3.Zero;
                return false;
            }

            Vec3 direction = rayEnd - rayStart;

            // The winding matters: the ray/triangle test is one-sided.
            bool struck;
            if (diagonal == 0)
            {
                // split along c1-c3, the anti-diagonal
                struck = RayTriangle(rayStart, direction, c0, c1, c3, out hit, out normal)
                      || RayTriangle(rayStart, direction, c1, c2, c3, out hit, out normal);
            }
            else
            {
                // split along c0-c2, the main diagonal
                struck = RayTriangle(rayStart, direction, c0, c1, c2, out hit, out normal)
                      || RayTriangle(rayStart, direction, c2, c3, c0, out hit, out normal);
            }

            if (!struck)
                return false;

            // The normal is normalised before it is returned, and that matters. The raw cross product
            // of two tile edges grows with the tile's slope -- on a 4:1 face it is (-4, 1, 0), length
            // 4.12 -- and every consumer reads dot products against it as if it were a unit vector.
            // In the swept solver an unnormalised normal makes the push-out several times too small,
            // so the move is not refused and the body creeps up a slope it should not climb.
            float length = normal.Length;
            if (length > 0f)
                normal = normal * (1f / length);

            return true;
        }

        /// <summary>
        /// The four corner positions of a tile and which way it is split.
        ///
        /// <para>
        /// <b>The diagonal is positional, not stored:</b> <c>(~tz ^ tx) &amp; 1</c>, i.e. tiles whose
        /// <c>tx</c> and <c>tz</c> share parity split one way and the rest the other. Getting it
        /// wrong disagrees with the rendered mesh on half of all tiles.
        /// </para>
        ///
        /// <para>
        /// <b>1 splits along c0-c2 (the main diagonal), 0 splits along c1-c3 (the anti-diagonal).</b>
        /// <see cref="GroundHeightAt"/> states the same mapping independently: it compares
        /// <c>localX</c> against <c>localZ</c> for diagonal 1 and against <c>tileSize - localZ</c> for
        /// diagonal 0.
        /// </para>
        /// </summary>
        internal void GetTileCorners(int tx, int tz, out Vec3 c0, out Vec3 c1, out Vec3 c2, out Vec3 c3, out int diagonal)
        {
            float tileSize = _tiles.TileSize;
            float x0 = tileSize * tx;
            float z0 = tileSize * tz;

            // The far samples clamp to the last row/column, so the map's edge tiles are flat rather
            // than reading past the end.
            int xp = Math.Min(tx + 1, _tiles.Width - 1);
            int zp = Math.Min(tz + 1, _tiles.Height - 1);

            c0 = new Vec3(x0, _tiles.SampleHeight(tx, tz), z0);
            c1 = new Vec3(x0 + tileSize, _tiles.SampleHeight(xp, tz), z0);
            c2 = new Vec3(x0 + tileSize, _tiles.SampleHeight(xp, zp), z0 + tileSize);
            c3 = new Vec3(x0, _tiles.SampleHeight(tx, zp), z0 + tileSize);

            diagonal = (~tz ^ tx) & 1;
        }

        // ---- the direct height query ----------------------------------------

        /// <summary>
        /// The interpolated terrain height under a world position, with the surface normal. This is
        /// the query <see cref="CalculateClosestPoint"/> is built on, and it is far cheaper than a ray
        /// cast because the tile is found by division rather than by walking.
        ///
        /// <para>
        /// It picks its triangle by comparing the position's offset within the tile against the
        /// split, which is a second, independent statement of the diagonal mapping in
        /// <see cref="GetTileCorners"/>: diagonal 1 compares <c>localX</c> against <c>localZ</c>
        /// (the main diagonal), diagonal 0 against <c>tileSize - localZ</c> (the anti-diagonal).
        /// </para>
        ///
        /// <para>
        /// The plane is solved from a reference corner, which is <c>c0</c> except for diagonal 0's
        /// far triangle — there it is <c>c1</c>, because <c>(c1,c2,c3)</c> does not contain
        /// <c>c0</c>.
        /// </para>
        ///
        /// <para>
        /// Returns 0 and a zero normal outside the map.
        /// </para>
        /// </summary>
        public float GroundHeightAt(Vec3 point, out Vec3 normal)
        {
            normal = Vec3.Zero;

            float tileSize = _tiles.TileSize;
            if (tileSize <= 0f)
                return 0f;

            if (point.X < 0f || tileSize * _tiles.Width <= point.X)
                return 0f;
            if (point.Z < 0f || tileSize * _tiles.Height <= point.Z)
                return 0f;

            int tx = (int)Math.Floor(point.X / tileSize);
            int tz = (int)Math.Floor(point.Z / tileSize);

            GetTileCorners(tx, tz, out Vec3 c0, out Vec3 c1, out Vec3 c2, out Vec3 c3, out int diagonal);

            float localX = point.X - c0.X;
            float localZ = point.Z - c0.Z;

            Vec3 reference;
            Vec3 edgeA;
            Vec3 edgeB;

            if (diagonal == 0)
            {
                if (tileSize - localZ < localX)
                {
                    // the far triangle (c1, c2, c3) — the reference corner moves to c1
                    reference = c1;
                    edgeA = c1 - c3;
                    edgeB = c1 - c2;
                }
                else
                {
                    reference = c0;
                    edgeA = c0 - c3;
                    edgeB = c0 - c1;
                }
            }
            else
            {
                if (localX <= localZ)
                {
                    reference = c0;
                    edgeA = c0 - c3;
                    edgeB = c0 - c2;
                }
                else
                {
                    reference = c0;
                    edgeA = c0 - c2;
                    edgeB = c0 - c1;
                }
            }

            Vec3 n = Vec3.Cross(edgeA, edgeB);
            if (n.Y == 0f)
                return reference.Y;

            float height = reference.Y
                - ((point.X - reference.X) * n.X + (point.Z - reference.Z) * n.Z) / n.Y;

            float length = n.Length;
            normal = length > 0f ? n * (1f / length) : Vec3.Zero;
            return height;
        }

        // ---- ray / triangle --------------------------------------------------

        /// <summary>
        /// A one-sided ray/triangle test.
        ///
        /// <para>
        /// It requires <c>dot(direction, cross(e0, e1)) &gt; 0</c>, i.e. the triangle must face
        /// <b>away</b> from the ray, and then returns the <b>negated</b> cross product as the normal —
        /// so the normal handed back always opposes the ray. For a downward probe on upward-facing
        /// terrain that yields the expected upward normal.
        /// </para>
        ///
        /// <para>
        /// The inside test is two edge-cross dot products against <see cref="EdgeEpsilon"/>, not
        /// three; the first cross is reused as the reference for both.
        /// </para>
        /// </summary>
        internal static bool RayTriangle(Vec3 origin, Vec3 direction, Vec3 v0, Vec3 v1, Vec3 v2, out Vec3 hit, out Vec3 normal)
        {
            // Written out per component: this is the innermost test of every ground probe. Each line
            // is the Vec3 operation it stands for, in the same order, so the floats are the same.
            hit = Vec3.Zero;

            // e0 = v1 - v0, e1 = v2 - v0, n = Cross(e0, e1)
            float e0x = v1.X - v0.X, e0y = v1.Y - v0.Y, e0z = v1.Z - v0.Z;
            float e1x = v2.X - v0.X, e1y = v2.Y - v0.Y, e1z = v2.Z - v0.Z;
            float nx = e0y * e1z - e0z * e1y;
            float ny = e0z * e1x - e0x * e1z;
            float nz = e0x * e1y - e0y * e1x;
            normal = new Vec3(nx, ny, nz);

            float denominator = direction.X * nx + direction.Y * ny + direction.Z * nz;
            if (denominator <= 0f)
                return false;

            // t = Dot(v0 - origin, n) / denominator; hit = origin + direction * t
            float t = ((v0.X - origin.X) * nx + (v0.Y - origin.Y) * ny + (v0.Z - origin.Z) * nz) / denominator;
            float hx = origin.X + direction.X * t;
            float hy = origin.Y + direction.Y * t;
            float hz = origin.Z + direction.Z * t;
            hit = new Vec3(hx, hy, hz);

            return EdgeTests(hx, hy, hz, v0, v1, v2, e0x, e0y, e0z, e1x, e1y, e1z, nx, ny, nz, ref normal);
        }

        /// <summary>
        /// The two edge tests that end <see cref="RayTriangle"/>, given the hit and the edges it
        /// already built. Sets <paramref name="normal"/> to <c>-n</c> on success.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal static bool EdgeTests(
            float hx, float hy, float hz, Vec3 v0, Vec3 v1, Vec3 v2,
            float e0x, float e0y, float e0z, float e1x, float e1y, float e1z,
            float nx, float ny, float nz, ref Vec3 normal)
        {
            // reference = Cross(e0, hit - v0)
            float px = hx - v0.X, py = hy - v0.Y, pz = hz - v0.Z;
            float rx = e0y * pz - e0z * py;
            float ry = e0z * px - e0x * pz;
            float rz = e0x * py - e0y * px;

            // Dot(Cross(v2 - v1, hit - v1), reference)
            float ax = v2.X - v1.X, ay = v2.Y - v1.Y, az = v2.Z - v1.Z;
            float qx = hx - v1.X, qy = hy - v1.Y, qz = hz - v1.Z;
            float cx = ay * qz - az * qy;
            float cy = az * qx - ax * qz;
            float cz = ax * qy - ay * qx;
            if (cx * rx + cy * ry + cz * rz < EdgeEpsilon)
                return false;

            // Dot(Cross(hit - v2, e1), reference)
            float sx = hx - v2.X, sy = hy - v2.Y, sz = hz - v2.Z;
            cx = sy * e1z - sz * e1y;
            cy = sz * e1x - sx * e1z;
            cz = sx * e1y - sy * e1x;
            if (cx * rx + cy * ry + cz * rz < EdgeEpsilon)
                return false;

            normal = new Vec3(-nx, -ny, -nz);
            return true;
        }

        // ---- veto -------------------------------------------------------------

        /// <summary>
        /// <b>Not implemented:</b> nothing is vetoed, so the ground clamp's retry loop never runs.
        /// Deliberately a no-op rather than a guess at what the terrain rejects.
        /// </summary>
        public bool VetoPosition(ref Vec3 position, object locality, Vec3 previous) => false;

        /// <summary>
        /// Drops the point onto the ground (<see cref="GroundHeightAt"/>). At or above the ground the
        /// <see cref="Child"/> is asked too, and the <b>higher</b> of the two answers wins — that is
        /// what lets a statel floor hold a body up and not just stop a ray.
        ///
        /// <para>
        /// <b>Not implemented:</b> leaving the output unwritten when the point is outside the cell
        /// grid, and clamping the result to <c>liquidHeight - 1.2</c>. There is no liquid yet, and this
        /// always writes a result.
        /// </para>
        /// </summary>
        public void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality)
        {
            float groundY = GroundHeightAt(point, out normal);
            closest = new Vec3(point.X, groundY, point.Z);

            // Below the ground the terrain answers alone; at or above it the child is asked and the
            // HIGHER of the two wins (a tie keeps the terrain).
            if (Child == null || point.Y < groundY)
                return;

            Child.CalculateClosestPoint(point, out Vec3 childClosest, out Vec3 childNormal, locality);

            if (childClosest.Y > groundY)
            {
                closest = childClosest;
                normal = childNormal;
            }
        }
    }
}

using System;
using System.Collections.Generic;

namespace N3Lite.Liquids
{
    /// <summary>
    /// A dungeon room's own liquid: its water triangles, kept in room space, and the test that says how
    /// high the surface is over a point.
    ///
    /// <para>
    /// Built once from the room's water meshes (<see cref="LiquidBuilder.ReadMeshes"/>): odd (sloped)
    /// types are left out, the rest are kept as they are, shifted by (1000, 0, 1000), with no cutting and
    /// no tests for degenerate pieces. A triangle whose normal faces down has the normal flipped and its
    /// second and third corners swapped. The room's turn is <b>not</b> applied to the triangles (it is
    /// multiplied by zero); instead the test turns the point the other way into room space.
    /// </para>
    /// </summary>
    public sealed class RoomLiquid
    {
        /// <summary>The shift room-space triangles and points are given.</summary>
        public const float RoomShift = 1000f;

        /// <summary>How far under the room's origin a miss answers.</summary>
        public const float MissDepth = 10000f;

        /// <summary>A quarter turn, as the float the test uses (<c>-π/2</c> per turn).</summary>
        const double QuarterTurn = -1.5707963705062866;

        readonly LiquidTriangle[] _triangles;

        public int Count => _triangles.Length;

        public RoomLiquid(IReadOnlyList<LiquidBuilder.MeshTriangle> triangles)
        {
            var kept = new List<LiquidTriangle>(triangles?.Count ?? 0);
            if (triangles != null)
            {
                for (int i = 0; i < triangles.Count; i++)
                {
                    LiquidTriangle t = Make(triangles[i]);
                    if (t != null)
                        kept.Add(t);
                }
            }
            _triangles = kept.ToArray();
        }

        /// <summary>
        /// The depth from a room mesh's flags: as <see cref="LiquidBuilder.DepthOf"/>, but the bits above
        /// the type are taken unsigned.
        /// </summary>
        public static float DepthOf(int flags)
        {
            uint bits = unchecked((uint)flags) >> 5;
            return bits == 0 ? LiquidBuilder.NoDepth : (float)(bits / 10.0);
        }

        static LiquidTriangle Make(LiquidBuilder.MeshTriangle m)
        {
            if ((m.Flags & 1) != 0)
                return null;

            var shift = new Vec3(RoomShift, 0f, RoomShift);
            Vec3 v0 = m.V0 + shift;
            Vec3 v1 = m.V1 + shift;
            Vec3 v2 = m.V2 + shift;

            // The unit normal of the unshifted corners; a zero one becomes NaN and never matches.
            Vec3 n = Vec3.Cross(m.V1 - m.V0, m.V2 - m.V0);
            float lengthSquared = n.X * n.X + n.Y * n.Y + n.Z * n.Z;
            float length = (float)Math.Sqrt(lengthSquared);
            float inverse = 1f / length;
            float scale = inverse * 1f;
            n = new Vec3(n.X * scale, n.Y * scale, n.Z * scale);

            if (n.Y < 0f)
            {
                n = new Vec3(-n.X, -n.Y, -n.Z);
                Vec3 t = v1;
                v1 = v2;
                v2 = t;
            }

            return new LiquidTriangle(LiquidBuilder.TypeOf(m.Flags), DepthOf(m.Flags), v0, v1, v2, n);
        }

        /// <summary>
        /// The surface's world height over <paramref name="point"/> for a room at <paramref name="origin"/>
        /// turned <paramref name="quarterTurns"/> times, with its normal, type and depth; or the origin's
        /// height less <see cref="MissDepth"/> when no triangle is over it. The first triangle over the
        /// point decides.
        ///
        /// <para>
        /// The normal is put through the same turn as the point rather than back the other way, so on a
        /// room turned an odd number of times a sloped surface's normal comes out mirrored in X and Z;
        /// a flat one is unaffected.
        /// </para>
        /// </summary>
        public float SurfaceHeight(Vec3 point, Vec3 origin, int quarterTurns, out Vec3 normal, out int type, out float depth)
        {
            normal = Vec3.Zero;
            type = 0;
            depth = 0f;

            Turn(quarterTurns, out float c, out float s);

            float dx = point.X - origin.X;
            float dy = point.Y - origin.Y;
            float dz = point.Z - origin.Z;
            var local = new Vec3(
                (float)(dx * (double)c + dz * (double)s) + RoomShift,
                dy,
                (float)(dz * (double)c - dx * (double)s) + RoomShift);

            for (int i = 0; i < _triangles.Length; i++)
            {
                LiquidTriangle t = _triangles[i];
                float height = t.SurfaceHeight(local);
                if (!(0f < height))
                    continue;

                Vec3 n = t.Normal;
                normal = new Vec3(
                    (float)(n.X * (double)c + n.Z * (double)s),
                    n.Y,
                    (float)(n.Z * (double)c - n.X * (double)s));
                type = t.Type;
                depth = t.Depth;
                return (float)(origin.Y + (double)height);
            }

            return (float)(origin.Y - (double)MissDepth);
        }

        /// <summary>
        /// The room's liquid test as the ground query asks it: the surface height when one is over the
        /// point, above 0 and with its bottom at or under <paramref name="limit"/>; otherwise
        /// <see cref="LiquidZones.NoLiquid"/>.
        /// </summary>
        public float PerformLiquidCollisionTest(Vec3 point, out Vec3 normal, out int type, float limit, Vec3 origin, int quarterTurns)
        {
            float height = SurfaceHeight(point, origin, quarterTurns, out normal, out type, out float depth);
            if (0f < height && height - depth <= limit)
                return height;
            return LiquidZones.NoLiquid;
        }

        /// <summary>The cosine and sine of <c>quarterTurns · -π/2</c>, each stored as a float.</summary>
        static void Turn(int quarterTurns, out float c, out float s)
        {
            float angle = (float)(unchecked((uint)quarterTurns) * QuarterTurn);
            c = (float)Math.Cos(angle);
            s = (float)Math.Sin(angle);
        }
    }
}

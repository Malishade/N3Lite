using System;
using System.Collections.Generic;

namespace N3Lite.Liquids
{
    /// <summary>
    /// A playfield's water meshes turned into the triangles the liquid test checks.
    ///
    /// <para>
    /// Two steps. <see cref="ReadMeshes"/> turns each stored mesh into plain triangles the way the
    /// playfield reader does: a mesh with flag bit 0 set, or with exactly three corners, is an indexed
    /// triangle list; any other mesh is a fan round the average of its corners. <see cref="Create"/>
    /// then turns one triangle into collision triangles: odd liquid types (the sloped ones) get none,
    /// even types are cut into four at their edge midpoints until each piece is small enough, and each
    /// piece is kept unless it is off the map, has two equal corners or has no normal.
    /// </para>
    /// </summary>
    public static class LiquidBuilder
    {
        /// <summary>Twice a triangle's area above which it is cut into four.</summary>
        public const float SplitArea = 10000f;

        /// <summary>The depth a mesh with no depth bits gets.</summary>
        public const float NoDepth = 100000f;

        /// <summary>The off-map test's grid: 60 x 60 cells of 100 units.</summary>
        const double CellSize = 100.0;
        const int CellsX = 60;
        const uint LastCell = 0xe0f;

        /// <summary>A triangle of a water mesh with the mesh's flags.</summary>
        public readonly struct MeshTriangle
        {
            public readonly Vec3 V0;
            public readonly Vec3 V1;
            public readonly Vec3 V2;
            public readonly int Flags;

            public MeshTriangle(Vec3 v0, Vec3 v1, Vec3 v2, int flags)
            {
                V0 = v0;
                V1 = v1;
                V2 = v2;
                Flags = flags;
            }
        }

        /// <summary>A stored water mesh: its flags, its corners and its index triples.</summary>
        public readonly struct Mesh
        {
            public readonly int Flags;
            public readonly IReadOnlyList<Vec3> Vertices;

            /// <summary>Three per triangle, in the order they are stored.</summary>
            public readonly IReadOnlyList<int> Indices;

            public Mesh(int flags, IReadOnlyList<Vec3> vertices, IReadOnlyList<int> indices)
            {
                Flags = flags;
                Vertices = vertices;
                Indices = indices;
            }
        }

        /// <summary>
        /// The liquid type of a mesh's flags: the low five bits.
        /// </summary>
        public static int TypeOf(int flags) => flags & 0x1f;

        /// <summary>
        /// The liquid's depth from a mesh's flags: the bits above the type, in tenths. A mesh that
        /// gives 0 is treated as bottomless (<see cref="NoDepth"/>).
        /// </summary>
        public static float DepthOf(int flags)
        {
            float depth = (float)((flags >> 5) / 10.0);
            return depth == 0f ? NoDepth : depth;
        }

        /// <summary>
        /// Every mesh's triangles, in order. The total is capped at the sum over the meshes of the
        /// corner count, less two for each indexed mesh (flag bit 0); an indexed mesh with more
        /// triangles than that allows has the excess dropped.
        /// </summary>
        public static List<MeshTriangle> ReadMeshes(IReadOnlyList<Mesh> meshes)
        {
            var triangles = new List<MeshTriangle>();
            if (meshes == null)
                return triangles;

            long cap = 0;
            for (int m = 0; m < meshes.Count; m++)
            {
                int count = meshes[m].Vertices?.Count ?? 0;
                cap += (meshes[m].Flags & 1) != 0 ? count - 2 : count;
            }

            for (int m = 0; m < meshes.Count; m++)
            {
                Mesh mesh = meshes[m];
                IReadOnlyList<Vec3> v = mesh.Vertices;
                int count = v?.Count ?? 0;

                if ((mesh.Flags & 1) == 0 && count != 3)
                {
                    if (count == 0)
                        continue;

                    // A fan round the corners' average: (centre, previous corner, corner).
                    var sum = Vec3.Zero;
                    for (int i = 0; i < count; i++)
                        sum = new Vec3(sum.X + v[i].X, sum.Y + v[i].Y, sum.Z + v[i].Z);
                    Vec3 centre = sum / count;

                    for (int i = 0; i < count; i++)
                    {
                        int previous = (i == 0 ? count : i) - 1;
                        triangles.Add(new MeshTriangle(centre, v[previous], v[i], mesh.Flags));
                    }
                }
                else
                {
                    IReadOnlyList<int> idx = mesh.Indices;
                    int n = idx == null ? 0 : idx.Count / 3;
                    for (int i = 0; i < n; i++)
                    {
                        triangles.Add(new MeshTriangle(
                            v[idx[i * 3] & 0xffff], v[idx[i * 3 + 1] & 0xffff], v[idx[i * 3 + 2] & 0xffff],
                            mesh.Flags));
                    }
                }
            }

            if (triangles.Count > cap)
                triangles.RemoveRange((int)Math.Max(0, cap), triangles.Count - (int)Math.Max(0, cap));
            return triangles;
        }

        /// <summary>
        /// One mesh triangle into collision triangles, appended to <paramref name="into"/> in the order
        /// the cut makes them.
        /// </summary>
        public static void Create(MeshTriangle triangle, List<LiquidTriangle> into)
            => Create(TypeOf(triangle.Flags), DepthOf(triangle.Flags), triangle.V0, triangle.V1, triangle.V2, into);

        public static void Create(int type, float depth, Vec3 v0, Vec3 v1, Vec3 v2, List<LiquidTriangle> into)
        {
            if (into == null || v0.Equals(v1) || v1.Equals(v2) || v2.Equals(v0))
                return;

            // The sloped (odd) types are drawn as cascades but never become collision triangles.
            if ((type & 1) != 0)
                return;

            Split(type, depth, v0, v1, v2, into);
        }

        /// <summary>
        /// Cut a triangle into four at its edge midpoints while twice its area is over
        /// <see cref="SplitArea"/>. Each midpoint is taken from the corner the piece starts at, so the
        /// same midpoint can differ in its last bit between pieces.
        /// </summary>
        static void Split(int type, float depth, Vec3 v0, Vec3 v1, Vec3 v2, List<LiquidTriangle> into)
        {
            Vec3 cross = Vec3.Cross(v1 - v0, v2 - v0);
            float lengthSquared = cross.X * cross.X + cross.Y * cross.Y + cross.Z * cross.Z;
            float doubleArea = (float)Math.Sqrt(lengthSquared);

            if (SplitArea < doubleArea)
            {
                Split(type, depth, v0, Mid(v1, v0), Mid(v2, v0), into);
                Split(type, depth, v1, Mid(v2, v1), Mid(v0, v1), into);
                Split(type, depth, v2, Mid(v0, v2), Mid(v1, v2), into);
                Split(type, depth, Mid(v1, v0), Mid(v2, v1), Mid(v0, v2), into);
                return;
            }

            Keep(type, depth, v0, v1, v2, into);
        }

        /// <summary>Halfway from <paramref name="from"/> to <paramref name="to"/>: <c>(to - from) / 2 + from</c>.</summary>
        static Vec3 Mid(Vec3 to, Vec3 from)
        {
            Vec3 half = (to - from) / 2f;
            return new Vec3(from.X + half.X, from.Y + half.Y, from.Z + half.Z);
        }

        /// <summary>
        /// The tests a piece must pass to become a collision triangle, then the triangle itself.
        /// </summary>
        static void Keep(int type, float depth, Vec3 v0, Vec3 v1, Vec3 v2, List<LiquidTriangle> into)
        {
            // Off the map: the centre must fall in the 60 x 60 grid of 100-unit cells.
            float cz = (v1.Z + v0.Z + v2.Z) / 3f;
            int row = (int)(cz / -CellSize);
            float cx = (v1.X + v0.X + v2.X) / 3f;
            int column = (int)(cx / CellSize);
            if (unchecked((uint)(column - row * CellsX)) > LastCell)
                return;

            if (v0.Equals(v1) || v1.Equals(v2) || v2.Equals(v0))
                return;

            Vec3 normal = UpNormal(v0, ref v1, ref v2);
            if (normal.X == 0f && normal.Y == 0f && normal.Z == 0f)
                return;

            if ((type & 1) != 0)
                return;

            into.Add(new LiquidTriangle(type, depth, v0, v1, v2, normal));
        }

        /// <summary>
        /// The unit normal <c>(v1 - v0) x (v2 - v0)</c>, with <paramref name="v1"/> and
        /// <paramref name="v2"/> swapped first when it would face down. A normal with every component
        /// under 1e-5 is zero.
        /// </summary>
        static Vec3 UpNormal(Vec3 v0, ref Vec3 v1, ref Vec3 v2)
        {
            Vec3 n = Vec3.Cross(v1 - v0, v2 - v0);
            if (n.Y < 0f)
            {
                Vec3 t = v1;
                v1 = v2;
                v2 = t;
                n = Vec3.Cross(v1 - v0, v2 - v0);
            }

            if (Math.Abs(n.X) < 1e-5f && Math.Abs(n.Y) < 1e-5f && Math.Abs(n.Z) < 1e-5f)
                return Vec3.Zero;

            float lengthSquared = n.X * n.X + n.Y * n.Y + n.Z * n.Z;
            float length = (float)Math.Sqrt(lengthSquared);
            float inverse = 1f / length;
            return new Vec3(n.X * inverse, n.Y * inverse, n.Z * inverse);
        }
    }
}

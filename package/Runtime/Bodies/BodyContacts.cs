using System.Collections.Generic;

namespace N3Lite.Bodies
{
    /// <summary>Body-against-body contact between <see cref="ContactBody"/> spheres, run once per frame.</summary>
    public static class BodyContacts
    {
        /// <summary>Bodies further apart than this on X or on Z are not checked.</summary>
        public const float PairReach = 10f;

        /// <summary>
        /// Every pair of <paramref name="bodies"/> once, as <see cref="Collide"/>. The contacts a pair
        /// records do not depend on which body comes first.
        /// </summary>
        public static void Run(IReadOnlyList<ContactBody> bodies)
        {
            int count = bodies.Count;
            if (count < SmallRun)
            {
                for (int i = 0; i < count; i++)
                    for (int j = i + 1; j < count; j++)
                        Collide(bodies[i], bodies[j]);
                return;
            }

            // The same pairs in the same order (i, then each j > i ascending), found through a grid of
            // PairReach cells on X/Z instead of trying all of them: bodies two or more cells apart are
            // further than PairReach apart on X or Z, which Check refuses anyway.
            foreach (List<int> cell in s_cells.Values)
            {
                cell.Clear();
                s_cellPool.Push(cell);
            }
            s_cells.Clear();

            if (s_keys.Length < count)
                s_keys = new long[count * 2];

            for (int i = 0; i < count; i++)
            {
                Vec3 p = bodies[i].Position;
                long key = CellKey(CellOf(p.X), CellOf(p.Z));
                s_keys[i] = key;
                if (!s_cells.TryGetValue(key, out List<int> cell))
                {
                    cell = s_cellPool.Count > 0 ? s_cellPool.Pop() : new List<int>();
                    s_cells[key] = cell;
                }
                cell.Add(i);
            }

            for (int i = 0; i < count; i++)
            {
                long key = s_keys[i];
                int cx = (int)(key >> 32), cz = (int)key;
                s_candidates.Clear();
                for (int dx = -1; dx <= 1; dx++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (!s_cells.TryGetValue(CellKey(cx + dx, cz + dz), out List<int> cell))
                            continue;
                        for (int k = 0; k < cell.Count; k++)
                        {
                            if (cell[k] > i)
                                s_candidates.Add(cell[k]);
                        }
                    }
                }

                s_candidates.Sort();
                ContactBody a = bodies[i];
                for (int k = 0; k < s_candidates.Count; k++)
                    Collide(a, bodies[s_candidates[k]]);
            }
        }

        // Below this many bodies the plain all-pairs loop is as cheap as the grid.
        const int SmallRun = 32;

        static readonly Dictionary<long, List<int>> s_cells = new Dictionary<long, List<int>>();
        static readonly Stack<List<int>> s_cellPool = new Stack<List<int>>();
        static readonly List<int> s_candidates = new List<int>();
        static long[] s_keys = new long[64];

        // A non-finite coordinate goes to one far-off cell: such a body never touched anything (its sphere
        // test fails), so leaving it out of the near cells changes no contact.
        static int CellOf(float v)
        {
            // Cells a little wider than PairReach, so rounding at a cell edge can never put a pair within
            // reach two cells apart.
            float c = v / (PairReach * 1.001f);
            if (!(c > -1e9f && c < 1e9f))
                return int.MinValue + 2;
            return (int)System.Math.Floor(c);
        }

        static long CellKey(int cx, int cz) => ((long)cx << 32) | (uint)cz;

        /// <summary>
        /// A pair where either body is <see cref="ContactMode.Excluded"/> is skipped; otherwise, when at
        /// least one of them is not <see cref="ContactMode.Passive"/>, the pair is checked.
        /// </summary>
        public static void Collide(ContactBody a, ContactBody b)
        {
            if (a.Mode == ContactMode.Excluded || b.Mode == ContactMode.Excluded)
                return;

            if (a.Mode != ContactMode.Passive || b.Mode != ContactMode.Passive)
                Check(a, b);
        }

        /// <summary>
        /// Nothing unless one of the two wants contacts; nothing past <see cref="PairReach"/> on X or Z;
        /// then the sphere test, and on contact a body that wants contacts is told about one that
        /// tells. Returns whether the spheres touch.
        /// </summary>
        public static bool Check(ContactBody a, ContactBody b)
        {
            if (!a.WantsContacts && !b.WantsContacts)
                return false;

            float dx = a.Position.X - b.Position.X;
            float dz = a.Position.Z - b.Position.Z;
            if (dx > PairReach || dx < -PairReach || dz > PairReach || dz < -PairReach)
                return false;

            if (!a.HasSphere || !b.HasSphere)
                return false;

            if (!SpheresTouch(a.SphereCenter, a.SphereRadius, b.SphereCenter, b.SphereRadius))
                return false;

            if (b.TellsContacts && a.WantsContacts)
                a.AddContact(b);

            if (a.TellsContacts && b.WantsContacts)
                b.AddContact(a);

            return true;
        }

        /// <summary>
        /// The squared distance against the squared radius sum; touching counts. The differences, the
        /// radius sum, each squared term and the final sum are rounded to float; the running sum and
        /// the squared radius sum are not.
        /// </summary>
        public static bool SpheresTouch(Vec3 a, float radiusA, Vec3 b, float radiusB)
        {
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            float dz = a.Z - b.Z;
            float reach = radiusA + radiusB;

            float dx2 = dx * dx;
            float dy2 = dy * dy;
            float dz2 = dz * dz;
            float sum = (float)((double)dx2 + dy2 + dz2);

            return sum <= (double)reach * reach;
        }
    }
}

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
            for (int i = 0; i < bodies.Count; i++)
                for (int j = i + 1; j < bodies.Count; j++)
                    Collide(bodies[i], bodies[j]);
        }

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

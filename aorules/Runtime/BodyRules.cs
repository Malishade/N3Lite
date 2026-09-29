using N3Lite;
using N3Lite.Bodies;

namespace N3Lite.AORules
{
    /// <summary>
    /// How the game sets up a character's or an item's <see cref="ContactBody"/>: its contact flags
    /// from the Flags stat and its sphere from the mesh's torso sphere.
    /// </summary>
    public static class BodyRules
    {
        /// <summary>Flags (stat 0) bit: the body tells bodies it touches.</summary>
        public const int FlagTellsContacts = 0x200;

        /// <summary>Flags bit: the body wants to be told about contacts.</summary>
        public const int FlagWantsContacts = 0x80000;

        /// <summary>Flags bit: the body pushes bodies apart.</summary>
        public const int FlagRepulsive = 0x2000;

        /// <summary>The three contact bits of the Flags stat; characters and items read them the same way.</summary>
        public static void ApplyFlags(ContactBody body, int flags)
        {
            body.TellsContacts = (flags & FlagTellsContacts) != 0;
            body.WantsContacts = (flags & FlagWantsContacts) != 0;
            body.Repulsive = (flags & FlagRepulsive) != 0;
        }

        /// <summary>
        /// The game's numeric body collision mode as a <see cref="ContactMode"/>: 0 passive, 1
        /// excluded, anything else active. Bodies default to 2.
        /// </summary>
        public static ContactMode ModeFromNumber(int mode)
            => mode == 0 ? ContactMode.Passive : mode == 1 ? ContactMode.Excluded : ContactMode.Active;

        /// <summary>The body scale from stat 360, a percentage; a body that never had it keeps 1.</summary>
        public static float BodyScaleFromStat(int? stat360) => stat360.HasValue ? stat360.Value / 100f : 1f;

        /// <summary>The sphere for a body wearing a plain mesh, from the mesh's torso sphere.</summary>
        public static void SetMeshSphere(ContactBody body, Vec3 meshCenter, float meshRadius, float bodyScale)
            => SetSphere(body, meshCenter.Y, meshRadius, bodyScale);

        /// <summary>
        /// The same for a body wearing a CAT mesh. Until the CAT mesh has loaded its radius reads 1 and
        /// the centre stays at (0, 0.5, 0).
        /// </summary>
        public static void SetCatMeshSphere(ContactBody body, bool meshLoaded, Vec3 catCenter, float catRadius, float bodyScale)
        {
            float centerY = meshLoaded ? catCenter.Y : 0.5f;
            float radius = meshLoaded ? catRadius : 1f;
            SetSphere(body, centerY, radius, bodyScale);
        }

        /// <summary>
        /// A negative radius becomes 0.5; the centre keeps only its height, scaled (x and z are
        /// multiplied by 0); the radius is scaled.
        /// </summary>
        static void SetSphere(ContactBody body, float centerY, float radius, float bodyScale)
        {
            if (radius < 0f)
                radius = 0.5f;

            body.SphereOffset = new Vec3(bodyScale * 0f, centerY * bodyScale, bodyScale * 0f);
            body.SphereRadius = bodyScale * radius;
            body.HasSphere = true;
        }
    }
}

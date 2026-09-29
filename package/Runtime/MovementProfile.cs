namespace N3Lite
{
    /// <summary>
    /// How a character may move right now: its speeds and what it is allowed to do. The game decides
    /// these — from stats, a stance, a mount, anything — and hands them to
    /// <see cref="N3CharVehicle.SetProfile"/>.
    /// </summary>
    public struct MovementProfile
    {
        /// <summary>Top speed driving forward, m/s.</summary>
        public float ForwardSpeed;

        /// <summary>Top speed backing up, m/s.</summary>
        public float ReverseSpeed;

        /// <summary>Strafe speed, m/s.</summary>
        public float StrafeSpeed;

        /// <summary>Strafe speed while backing up, m/s.</summary>
        public float ReverseStrafeSpeed;

        /// <summary>
        /// False refuses forward and backward drive. Strafe and turn still work; a following NPC
        /// brakes to a halt.
        /// </summary>
        public bool CanDrive;

        /// <summary>
        /// False ignores input and refuses jumps, and entering such a profile clears the flags and
        /// the path — a seated character, say.
        /// </summary>
        public bool AcceptsInput;

        /// <summary>True turns gravity off.</summary>
        public bool Flying;

        /// <summary>5 m/s forward, 3 back, 2.5 strafing (1.5 backing up); free to move and jump.</summary>
        public static MovementProfile Default => new MovementProfile
        {
            ForwardSpeed = 5f,
            ReverseSpeed = 3f,
            StrafeSpeed = 2.5f,
            ReverseStrafeSpeed = 1.5f,
            CanDrive = true,
            AcceptsInput = true,
        };
    }
}

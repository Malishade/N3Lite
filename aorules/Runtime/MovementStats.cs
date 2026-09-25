namespace N3Lite.AORules
{
    /// <summary>
    /// The stats a character's movement reads, as their full (modified) values: the run speed that
    /// drives the speed curves, Strength (16), Agility (17) and GmLevel (215) for the jump height,
    /// and Scale (360) for the ceiling clamp. See <see cref="CharMovementRules"/>.
    ///
    /// <para>
    /// A client and a server must fill this from the same values — the buffed ones — or the two will
    /// disagree about how fast and how high a character moves.
    /// </para>
    /// </summary>
    public struct MovementStats
    {
        public int RunSpeed;
        public int Strength;
        public int Agility;
        public int GmLevel;

        /// <summary>
        /// Stat 224. An NPC follows only while bit 4 (<c>0x4</c>) is set (<c>10070503</c>); see
        /// <see cref="CharMovementRules.CanFollow"/>. What sets that bit is not traced.
        /// </summary>
        public int Features;

        /// <summary>Stat 360, in percent. Zero or less reads as 100.</summary>
        public int Scale;

        /// <summary><c>n3VisualDynel_t::GetBodyScale</c>: stat 360 / 100.</summary>
        public float BodyScale => Scale > 0 ? Scale / 100f : 1f;
    }
}

namespace N3Lite.Surfaces
{
    /// <summary>
    /// The world a vehicle collides against: the three queries ground contact uses.
    ///
    /// <para>
    /// <c>locality</c> identifies who is asking. Nothing reads it yet; it is an <see cref="object"/>
    /// until something needs it.
    /// </para>
    /// </summary>
    public interface ISurface
    {
        /// <summary>
        /// The nearest hit along the segment from <paramref name="start"/> to <paramref name="end"/>.
        ///
        /// <para>
        /// On a miss the terrain writes <c>(-1, -1, -1)</c> into <paramref name="hit"/> rather than
        /// leaving it alone, so a caller that reads it after a false return sees that sentinel.
        /// </para>
        /// </summary>
        bool GetLineIntersection(
            Vec3 start,
            Vec3 end,
            out Vec3 hit,
            out Vec3 normal,
            bool clipToBounds,
            object locality);

        /// <summary>
        /// Returns true when <paramref name="position"/> is <b>rejected</b>; the ground clamp then
        /// backs the move off and tries again.
        /// </summary>
        bool VetoPosition(ref Vec3 position, object locality, Vec3 previous);

        /// <summary>
        /// The ground under <paramref name="point"/>, which the ground clamp uses as its step-height
        /// reference.
        ///
        /// <para>
        /// Liquid (water height, normal and medium at the point) belongs here too, and is not
        /// implemented yet.
        /// </para>
        /// </summary>
        void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality);
    }
}

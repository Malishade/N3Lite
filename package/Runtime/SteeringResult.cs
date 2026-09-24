namespace LostEden.Vehicles
{
    /// <summary>
    /// What a steering behaviour asks the integrator to do with the vector it produced.
    ///
    /// The integrator reads steering on three channels, and each channel honours one value:
    /// <list type="bullet">
    ///   <item>longitudinal honours <see cref="Force"/>;</item>
    ///   <item>lateral honours <see cref="Lateral"/>;</item>
    ///   <item>turn honours <see cref="Turn"/>.</item>
    /// </list>
    /// Every channel treats <see cref="Halt"/> as "zero this channel", and the longitudinal one also
    /// halts the vehicle. Any other value leaves the channel's vector unused.
    /// </summary>
    public enum SteeringResult
    {
        /// <summary>
        /// No steering this step. Also what a behaviour returns when its force is absurdly large
        /// (<c>SteeringArrive</c> above 1e7).
        /// </summary>
        None = 0,

        /// <summary>Stop: the channel's vector is zeroed. On the longitudinal channel, also halts.</summary>
        Halt = 1,

        /// <summary>The vector is a force in newtons; the integrator clamps and integrates it.</summary>
        Force = 2,

        /// <summary>The vector is a velocity applied straight to position, bypassing the force path.</summary>
        Lateral = 3,

        /// <summary>The vector is a rotation axis whose length is the angular rate in radians/second.</summary>
        Turn = 4,
    }
}

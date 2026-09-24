namespace LostEden.Vehicles
{
    /// <summary>
    /// The camera mode, persisted as the integer preference <c>"PreferredCameraMode"</c>.
    ///
    /// 0 builds <see cref="CameraVehicleFirstPersonSim"/>, 1 the base <see cref="CameraVehicleSim"/>,
    /// 2 and 3 <see cref="CameraVehicleFixedThirdSim"/>. 2 and 3 differ only in
    /// <see cref="CameraVehicleFixedThirdSim.Frozen"/>: 2 steers (springy), 3 places the camera
    /// directly (rigid).
    ///
    /// Mode 0 is never written to the preference.
    /// </summary>
    public enum CameraViewMode
    {
        /// <summary>Camera sits at the eye; rotation is the character's times the mouse angles.</summary>
        FirstPerson = 0,

        /// <summary>
        /// Third person, steered by the base camera vehicle's own brain with obstacle sensors — the
        /// only mode the sensors run in.
        /// </summary>
        Trail = 1,

        /// <summary>
        /// Third person, physically steered to the ideal spot by <c>SteeringCamArrive</c> — mass,
        /// max force and arrive damping, so it lags and settles. The springy one.
        /// </summary>
        Rubber = 2,

        /// <summary>
        /// Third person, <b>not</b> steered: <c>CalcSteering</c> returns None and <c>DecideSnap</c>
        /// places the camera directly each time the heading is recomputed. Rigid.
        /// </summary>
        Lock = 3,
    }
}

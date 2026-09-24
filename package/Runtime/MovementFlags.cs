using System;

namespace N3Lite
{
    /// <summary>
    /// Movement input as flags — what an input layer or a network message produces.
    /// <see cref="N3CharVehicle"/> turns them into the vehicle's four axes.
    /// </summary>
    [Flags]
    public enum MovementFlags
    {
        None = 0,
        Forward = 1 << 0,
        Backward = 1 << 1,
        TurnLeft = 1 << 2,
        TurnRight = 1 << 3,
        StrafeLeft = 1 << 4,
        StrafeRight = 1 << 5,
        Jump = 1 << 6,
    }
}

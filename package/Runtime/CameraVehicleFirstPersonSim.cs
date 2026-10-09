using System;

namespace N3Lite
{
    /// <summary>
    /// The first-person camera (view mode 0).
    ///
    /// It does <b>no steering at all</b>: all three channels return
    /// <see cref="SteeringResult.None"/>. The camera is placed directly at the eye and rotated, which
    /// is why first person has none of the lag the third-person modes have. It has no surface either,
    /// so it never runs the occlusion solve.
    /// </summary>
    public class CameraVehicleFirstPersonSim : CameraVehicleSim
    {
        /// <summary>
        /// The mouse angles as a quaternion — yaw about world up, then pitch about local right.
        /// </summary>
        public Quat RotAngles { get; private set; } = Quat.Identity;

        /// <summary>The yaw last set, in radians. Kept so a delta can be added to it.</summary>
        public float Yaw { get; private set; }

        /// <summary>The pitch last set, in radians.</summary>
        public float Pitch { get; private set; }

        /// <summary>
        /// The camera's world rotation: the character's rotation composed with the mouse angles.
        /// </summary>
        public Quat ViewRotation => TargetRotation * RotAngles;

        /// <summary>
        /// Builds yaw about <c>(0,1,0)</c> and pitch about <c>(1,0,0)</c>, composes them, then
        /// re-places the camera.
        /// </summary>
        public void SetRotAngles(float pitchRadians, float yawRadians)
        {
            Pitch = pitchRadians;
            Yaw = yawRadians;

            Quat yaw = Quat.FromAxisAngle(Vec3.ReferenceUp, yawRadians);
            Quat pitch = Quat.FromAxisAngle(new Vec3(1f, 0f, 0f), pitchRadians);

            RotAngles = (yaw * pitch).Normalized;

            PlaceAtEye();
        }

        /// <summary>
        /// Puts the camera <b>at</b> the look target — the eye, not behind it — facing
        /// <see cref="ViewRotation"/>.
        /// </summary>
        public void PlaceAtEye()
        {
            SetRelPos(GetLookTargetPos());
            BodyRotation = ViewRotation;
        }

        /// <summary>
        /// <c>GetBackPush</c> (<c>N3 1001ed07</c>, <c>+0x1fc</c>): the eye offset's z as it was last set.
        /// </summary>
        public float BackPush { get; private set; }

        /// <summary>
        /// <c>CameraVehicleFirstPerson_t::SetEyeTargetLocalPos</c> (<c>N3 1001ed86</c>): stores the eye
        /// offset, keeps its z as <see cref="BackPush"/>, and puts the camera at the new eye
        /// (<c>SetRelPosRot(GetLookTargetPos(), ...)</c>).
        /// </summary>
        public override void SetEyeTargetLocalPos(Vec3 eye)
        {
            EyeTargetLocalPos = eye;
            BackPush = eye.Z;
            PlaceAtEye();
        }

        /// <summary>
        /// The reference forward taken through the mouse angles and then the character's rotation.
        /// </summary>
        public override bool VetoForward(out Vec3 forward)
        {
            forward = ViewRotation * Vec3.ReferenceForward;
            return true;
        }

        /// <summary>First person never steers.</summary>
        protected override SteeringResult CalcSteering(out Vec3 force)
        {
            force = Vec3.Zero;
            return SteeringResult.None;
        }

        /// <summary>First person has no surface, so no occlusion solve.</summary>
        public override void ForcedUpdate(bool recalcDistance) { }

        /// <summary><c>CameraVehicleFirstPerson_t::GetSurface</c> (<c>10007572</c>): none, so no surface alignment.</summary>
        protected override Surfaces.ISurface GetSurface() => null;

        /// <summary>
        /// The driver still ticks the vehicle, but with nothing to steer the only thing that
        /// matters is keeping the camera glued to the eye as the character moves.
        /// </summary>
        public override void Tick(float dt, float characterMaxSpeed)
        {
            PlaceAtEye();
        }
    }
}

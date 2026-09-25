using System;
using N3Lite.Surfaces;

namespace N3Lite
{
    /// <summary>
    /// A character's vehicle, driven by four input axes — a player's body. The camera vehicle sits on
    /// the same <see cref="VehicleSim"/> integrator.
    ///
    /// <para>
    /// <b>Movement input is four float axes, not a flag bitmask.</b>
    /// </para>
    /// </summary>
    public class CharVehicleSim : VehicleSim
    {
        // ---- the four input axes -------------------------------------------

        /// <summary>
        /// Positive drives forward, negative reverses, zero means no longitudinal steering at all. It
        /// is a <b>gate</b>, not a speed — the speed comes from <see cref="VehicleSim.MaxVel"/>.
        /// </summary>
        public float ForwardDrive;

        /// <summary>
        /// Unlike the others this <b>is</b> a speed in m/s: <c>sign(requested) * </c>
        /// <see cref="StrafeSpeed"/>. Use <see cref="SetStrafe"/>.
        /// </summary>
        public float Strafe;

        /// <summary>Radians per second about world Y.</summary>
        public float TurnRate;

        /// <summary>Along <b>world</b> up, not body up.</summary>
        public float Vertical;

        // ---- the speeds ---------------------------------------------------------

        /// <summary>The strafe speed in m/s. <see cref="SetStrafe"/> applies it with the requested sign.</summary>
        public float StrafeSpeed = 1f;

        /// <summary>
        /// Refuses longitudinal steering: a player's body gets no drive force at all, an NPC's brakes
        /// to a halt.
        /// </summary>
        public bool DriveLocked;

        // ---- the setters ----------------------------------------------------

        public void SetForwardDrive(float value) => ForwardDrive = value;

        public void SetTurnRate(float radiansPerSecond) => TurnRate = radiansPerSecond;

        public void SetVertical(float value) => Vertical = value;

        /// <summary>
        /// <c>Strafe = sign(requested) * StrafeSpeed</c>: the caller's magnitude is discarded, only its
        /// sign matters.
        /// </summary>
        public void SetStrafe(float requested)
            => Strafe = Sign(requested) * StrafeSpeed;

        /// <summary>+1, 0 or -1.</summary>
        internal static float Sign(float v) => v > 0f ? 1f : (v < 0f ? -1f : 0f);

        /// <summary>
        /// Sets the max velocity to <paramref name="maxSpeed"/> and derives the max force and brake
        /// distance from it. Call it whenever the speed changes.
        ///
        /// <para>
        /// The force is <c>2 * speed * mass</c>, which reaches full speed in half a second. The brake
        /// distance reduces exactly to <c>speed / 4</c>: <c>(v*v*m) / (2*v*m) * 0.5</c>. It is kept in
        /// the long form so the derivation stays visible.
        /// </para>
        /// </summary>
        public void UpdateMotionConstraints(float maxSpeed)
        {
            float mass = Mass;
            if (mass == 0f)
                mass = 10f;
            Mass = mass;

            float v = maxSpeed;
            if (v < 0.01f)
                v = 0.1f;

            float force = 2f * v * mass;
            if (force > 100000f)
                force = 100000f;

            float brake = (v * v * mass) / force * 0.5f;

            if (force > 10000f)
                force = 10000f;        // the effective cap is 10000, applied AFTER the brake maths

            MaxVel = v;
            MaxForce = force;
            SlowingDistance = brake;
        }

        // ---- the three steering channels ------------------------------------

        /// <summary>
        /// The longitudinal channel: forward or reverse on <see cref="ForwardDrive"/>.
        ///
        /// <para>
        /// <b>Not implemented:</b> arriving at a follow target, which needs the camera's targeting.
        /// </para>
        /// </summary>
        protected override SteeringResult CalcSteering(out Vec3 force)
        {
            force = Vec3.Zero;

            if (DriveLocked)
                return SteeringResult.None;

            if (ForwardDrive == 0f)
                return SteeringResult.None;

            return ForwardDrive > 0f
                ? SteeringForward(out force)
                : SteeringReverse(out force);
        }

        /// <summary>
        /// The lateral channel: <c>lateral = bodyRight * strafe + worldUp * vertical</c>. The up is
        /// world up, not affected by the body's orientation.
        /// </summary>
        protected override SteeringResult CalcLateralSteering(out Vec3 lateral)
        {
            lateral = Vec3.Zero;

            if (Strafe == 0f && Vertical == 0f)
                return SteeringResult.None;

            lateral = CalcBodyRight() * Strafe + Vec3.ReferenceUp * Vertical;
            return SteeringResult.Lateral;
        }

        /// <summary>
        /// The turn channel: an axis of <c>(0, turnRate, 0)</c>.
        ///
        /// <para>
        /// The integrator reads a <see cref="SteeringResult.Turn"/> result as an axis whose
        /// <b>length is the angular rate</b>, and rotates the <b>body</b> when standing still and the
        /// <b>velocity</b> when moving.
        /// </para>
        /// </summary>
        protected override SteeringResult CalcTurnSteering(out Vec3 turn)
        {
            turn = Vec3.Zero;

            if (TurnRate == 0f)
                return SteeringResult.None;

            turn = new Vec3(0f, TurnRate, 0f);
            return SteeringResult.Turn;
        }

        /// <summary>The body rotation applied to <c>(1,0,0)</c>.</summary>
        public Vec3 CalcBodyRight() => BodyRotation * new Vec3(1f, 0f, 0f);

        // ---- the jump --------------------------------------------------------

        /// <summary>
        /// The height of the jump in progress. Zero means none: <see cref="Jump"/> refuses while it is
        /// non-zero, and landing clears it.
        /// </summary>
        public float JumpHeight { get; private set; }

        /// <summary>
        /// The body's height. The ceiling clamp in <see cref="Jump"/> keeps this much clear above the
        /// position.
        /// </summary>
        public float BodyHeight = 2f;

        /// <summary>Raised from <see cref="OnLanded"/> when a jump in progress ends.</summary>
        public event Action JumpLanded;

        /// <summary>
        /// A moving body halted and its movement should stop. Stock applies <c>ForwardStop</c> here, or
        /// <c>BackwardStop</c> when <see cref="VehicleSim.Direction"/> is negative
        /// (<c>CharVehicle_t::OnHalt</c>, <c>1006f49d</c>). N3Lite has no movement status, so the owner
        /// applies it: the client to its own state, the server by sending it.
        /// </summary>
        public event Action Halted;

        /// <inheritdoc/>
        protected override void OnHalt() => RaiseHalted();

        protected void RaiseHalted() => Halted?.Invoke();

        /// <summary>
        /// Starts a jump of <paramref name="height"/>.
        /// Returns false when refused because a jump is already in progress.
        ///
        /// <para>
        /// In order: refuse unless <see cref="JumpHeight"/> is zero. Probe the surface straight up,
        /// from the position to the position plus <c>(0, 100, 0)</c>. On a hit the headroom is
        /// <c>hit.y - y - </c><see cref="BodyHeight"/>, floored at 0.1, and the height becomes the
        /// smaller of the two. Store it and launch at <c>sqrt(2 * height * |g|)</c> through <see cref="VehicleSim.Impact"/> as
        /// <c>(0, v * mass, 0)</c>, then <see cref="VehicleSim.EnableFalling"/>.
        /// </para>
        ///
        /// <para>
        /// That gate is the only one: <see cref="VehicleSim.Airborne"/> is not checked. A jump started
        /// in the air sets <see cref="JumpHeight"/> but <see cref="VehicleSim.Impact"/> drops the
        /// launch.
        /// </para>
        /// </summary>
        public bool Jump(float height)
        {
            if (JumpHeight != 0f)
                return false;

            ISurface surface = GetSurface();
            if (surface != null
                && surface.GetLineIntersection(
                    Position, Position + new Vec3(0f, 100f, 0f), out Vec3 hit, out _, false, null))
            {
                // Computed in double, rounded to float only once the body height is off.
                float headroom = (float)(((double)hit.Y - Position.Y) - BodyHeight);
                if (headroom < 0.1f)
                    headroom = 0.1f;
                if (headroom <= height)
                    height = headroom;
            }

            JumpHeight = height;

            float speed = MathF.Sqrt((height + height) * MathF.Abs(GravityAccel));
            Impact(new Vec3(0f, speed * Mass, 0f));
            EnableFalling();
            return true;
        }

        /// <summary>
        /// The landing notification from <see cref="VehicleSim.LandNow"/>. Clears
        /// <see cref="JumpHeight"/> and raises <see cref="JumpLanded"/> when a jump was in progress —
        /// that event is where the character's own state machine hooks in.
        /// </summary>
        protected override void OnLanded(float height)
        {
            bool wasJumping = JumpHeight != 0f;
            JumpHeight = 0f;
            if (wasJumping)
                JumpLanded?.Invoke();
        }

        // ---- the surface binding --------------------------------------------

        /// <summary>
        /// The world this vehicle collides against, set once the playfield's terrain is ready.
        ///
        /// <para>Null leaves <see cref="VehicleSim.EnsureSurfaceAlignment"/> a no-op: the body free-falls.</para>
        /// </summary>
        public ISurface Surface { get; set; }

        /// <inheritdoc/>
        protected override ISurface GetSurface() => Surface;
    }
}

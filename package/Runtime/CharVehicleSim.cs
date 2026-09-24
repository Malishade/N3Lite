using System;
using LostEden.Vehicles.Surfaces;

namespace LostEden.Vehicles
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

        // ---- the state the speed model keys off -----------------------------

        /// <summary>
        /// The movement state. States <b>1, 8 and 9 refuse longitudinal steering</b> outright, and 2,
        /// 3, 4, 5, 7 each have their own speed curve. 7 is Fly.
        ///
        /// <para>
        /// The names behind the other numbers are not known, so this is deliberately an <c>int</c>
        /// rather than an enum.
        /// </para>
        /// </summary>
        public int MovementState = 3;

        /// <summary>
        /// Selects the forward or backward curve within state 3: <b>2 means reverse</b>, anything else
        /// forward. Distinct from <see cref="VehicleSim.Direction"/>, the facing flip.
        /// </summary>
        public int CurveDirection = 1;

        /// <summary>The run-speed stat, which drives the speed curve.</summary>
        public float RunSpeedStat;

        /// <summary>The speed curve's base, stored before the stat is applied.</summary>
        public float SpeedBase { get; private set; }

        // ---- the setters ----------------------------------------------------

        public void SetForwardDrive(float value) => ForwardDrive = value;

        public void SetTurnRate(float radiansPerSecond) => TurnRate = radiansPerSecond;

        public void SetVertical(float value) => Vertical = value;

        /// <summary>
        /// <c>Strafe = sign(requested) * StrafeSpeed(state)</c>: the caller's magnitude is discarded,
        /// only its sign matters.
        /// </summary>
        public void SetStrafe(float requested)
            => Strafe = Sign(requested) * StrafeSpeed(MovementState);

        /// <summary>+1, 0 or -1.</summary>
        internal static float Sign(float v) => v > 0f ? 1f : (v < 0f ? -1f : 0f);

        // ---- the speed model --------------------------------------------------

        /// <summary>
        /// The per-state speed curve. Returns the max velocity for
        /// <see cref="UpdateMotionConstraints"/> and, halved, the strafe speed.
        /// </summary>
        struct Curve
        {
            public float Divisor;
            public float Base;
            public float Max;
            public float Min;
            public bool Constant;
        }

        static Curve CurveFor(int state, int direction)
        {
            switch (state)
            {
                case 3:
                    return direction == 2
                        ? new Curve { Divisor = 275f / 0.7f, Base = 3f, Max = 9.099999f, Min = 1.05f }
                        : new Curve { Divisor = 275f, Base = 5f, Max = 13f, Min = 1.5f };
                case 4:
                    return new Curve { Divisor = 275f / 0.625f, Base = 3f, Max = 8f, Min = 1.5f };
                case 7:
                    return new Curve { Divisor = 275f, Base = 7f, Max = 15f, Min = 1.5f };
                case 5:
                    return new Curve { Base = 1f, Constant = true };
                default:
                    // state 2 and everything unlisted
                    return new Curve { Base = 1.5f, Constant = true };
            }
        }

        /// <summary>
        /// The strafe speed: the forward curve <b>scaled by 0.5</b>, with a floor of 0.75 and no
        /// separate maximum — <c>clamp(0.5*stat/divisor + 0.5*base, 0.75, 0.5*max)</c>. For the run
        /// state that is <c>clamp(stat*0.5/275 + 2.5, 0.75, 6.5)</c>.
        /// </summary>
        public float StrafeSpeed(int state)
        {
            const float Scale = 0.5f;
            const float Floor = 0.75f;

            if (state == 2)
                return Math.Max(Floor, 1.5f);

            Curve curve = CurveFor(state, CurveDirection);
            if (curve.Constant)
                return Math.Max(Floor, curve.Base);

            float v = Scale * RunSpeedStat / curve.Divisor + Scale * curve.Base;
            float max = Scale * curve.Max;
            if (v > max)
                v = max;
            if (v < Floor)
                v = Floor;
            return v;
        }

        /// <summary>
        /// Recomputes mass, max velocity, max force and brake distance. Call it on every movement
        /// state or speed-stat change.
        ///
        /// <para>
        /// The brake distance reduces exactly to <c>maxVel / 4</c> in every branch:
        /// <c>(v*v*m) / (2*v*m) * 0.5</c>. It is kept in the long form so the derivation stays visible.
        /// </para>
        /// </summary>
        public void UpdateMotionConstraints()
        {
            float mass = Mass;
            if (mass == 0f)
                mass = 10f;
            Mass = mass;

            Curve curve = CurveFor(MovementState, CurveDirection);
            SpeedBase = curve.Base;

            float v;
            if (curve.Constant)
            {
                v = curve.Base;
                if (v < 0.01f)
                    v = 0.1f;
            }
            else
            {
                v = RunSpeedStat / curve.Divisor + curve.Base;
                if (v > curve.Max)
                    v = curve.Max;
                if (v < curve.Min)
                    v = curve.Min;
            }

            if (MovementState == 7)
                DisableFalling();      // Fly turns gravity off

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

            if (MovementState == 9 || MovementState == 8 || MovementState == 1)
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

        /// <summary>Whether the owning character is an NPC; an NPC's jump height is floored.</summary>
        public bool OwnerIsNpc;

        /// <summary>
        /// The owning character's body scale. The ceiling clamp in <see cref="Jump"/> subtracts twice
        /// this.
        /// </summary>
        public float OwnerBodyScale = 1f;

        /// <summary>Raised from <see cref="OnLanded"/> when a jump in progress ends.</summary>
        public event Action JumpLanded;

        /// <summary>
        /// A character's jump height from Strength (stat 16), Agility (17) and GmLevel (215):
        /// <c>(str + agi) / 200 + 1</c>, at least 0.5. Past 800 in total a non-GM counts as exactly
        /// 800 (<c>str = 800, agi = 0</c>).
        /// </summary>
        public static float JumpHeightFromStats(int strength, int agility, int gmLevel)
        {
            float str = strength;
            float agi = agility;
            if (800f < agi + str && gmLevel == 0)
            {
                str = 800f;
                agi = 0f;
            }

            float height = (float)((agi + str) / 200.0 + 1.0);
            if (height < 0.5f)
                height = 0.5f;
            return height;
        }

        /// <summary>
        /// Starts a jump of <paramref name="height"/> (usually <see cref="JumpHeightFromStats"/>).
        /// Returns false when refused because a jump is already in progress.
        ///
        /// <para>
        /// In order: refuse unless <see cref="JumpHeight"/> is zero. Probe the surface straight up,
        /// from the position to the position plus <c>(0, 100, 0)</c>. On a hit the headroom is
        /// <c>hit.y - y - 2 * bodyScale</c>, floored at 0.1, and the height becomes the smaller of the
        /// two. Store it; an NPC's stored value (<b>only</b> the stored one) is raised to 1.5. Launch
        /// at <c>sqrt(2 * height * |g|)</c> through <see cref="VehicleSim.Impact"/> as
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
                // Computed in double, rounded to float only once the body scale is off.
                float headroom = (float)(((double)hit.Y - Position.Y) - (OwnerBodyScale + OwnerBodyScale));
                if (headroom < 0.1f)
                    headroom = 0.1f;
                if (headroom <= height)
                    height = headroom;
            }

            JumpHeight = height;
            if (OwnerIsNpc && JumpHeight < 1.5f)
                JumpHeight = 1.5f;

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

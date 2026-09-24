using System;

namespace LostEden.Vehicles
{
    /// <summary>
    /// The camera's steering brain: it decides where the camera wants to be and hands a force to
    /// <see cref="VehicleSim"/>, which moves it. The host supplies the look target and the
    /// line-of-sight query.
    /// </summary>
    public class CameraVehicleSim : VehicleSim
    {
        /// <summary>
        /// The working follow distance is never allowed below this. The same figure is where the
        /// fixed third-person camera's facing blend starts.
        /// </summary>
        public const float MinFollowDistance = 0.9f;

        /// <summary>The follow distance before any preference is loaded.</summary>
        public const float DefaultFollowDistance = 5f;

        /// <summary>
        /// The halt radius the fixed third-person camera passes to <see cref="SteeringCamArrive"/>,
        /// so small that the camera effectively never halts.
        /// </summary>
        public const float ArriveHaltRadius = 0.01f;

        /// <summary>
        /// <see cref="UpdateMotionConstraints"/> reaches max speed in this long — max force is
        /// <c>mass * maxVel / 0.3</c>.
        /// </summary>
        public const float ForceReachTime = 0.3f;

        /// <summary>Below this the camera never slows down.</summary>
        public const float MinCatchUpSpeed = 2f;

        /// <summary>Speed ceiling while catching up to a distant target.</summary>
        public const float MaxCatchUpSpeed = 80f;

        /// <summary>Beyond this distance the camera starts speeding up.</summary>
        public const float CatchUpDistance = 6f;

        /// <summary>
        /// The camera's base speed is the character's <b>max</b> speed times this, so a faster
        /// character gets a faster camera.
        /// </summary>
        public const float BaseSpeedFromCharacterSpeed = 6f;

        /// <summary>Ceiling on that derived base speed.</summary>
        public const float MaxBaseSpeed = 16f;

        /// <summary>Subtracted before the square root in the catch-up curve.</summary>
        public const float CatchUpFalloff = 5f;

        /// <summary>
        /// <see cref="SteeringCamArrive"/> halts outright when a step is more than this many times the
        /// running average step — a hitch guard.
        /// </summary>
        public const float HitchStepRatio = 10f;

        // ---- factory settings ---------------------------------------------------

        /// <summary>
        /// The values the camera is built with: mass 20 — not the vehicle default of 50 — and falling
        /// and surface hug both off, so the camera has no gravity and is free to move vertically.
        /// </summary>
        public void ApplyDefaultSettings()
        {
            Mass = 20f;
            Velocity = Vec3.Zero;
            MaxForce = 600f;
            MaxVel = 15f;
            // Radius stays 0.01: the camera never runs the ground clamp that would reset it, and a
            // larger radius pushes the occlusion probe out and makes occlusion fire too eagerly.
            SlowingDistance = 15f;
            MaxSubStep = CameraSubStep;     // 0.05
            DisableFalling();
            DisableSurfaceHug();
        }

        // ---- target ---------------------------------------------------------

        /// <summary>
        /// Whether <c>GetSurface()</c> returns anything. The occlusion solve is gated on it, and the
        /// first-person camera has none, which is why first person never runs it.
        /// </summary>
        public bool HasSurface = true;

        /// <summary>The followed target's world position.</summary>
        public Vec3 TargetPosition;

        /// <summary>The followed target's rotation.</summary>
        public Quat TargetRotation = Quat.Identity;

        /// <summary>The eye offset in the target's local space (head height and so on).</summary>
        public Vec3 EyeTargetLocalPos;

        /// <summary>The working follow distance, clamped by <see cref="Update"/>.</summary>
        public float FollowDistance = DefaultFollowDistance;

        /// <summary>
        /// The zoom rate; non-zero diverts steering to <see cref="ZoomSteer"/>. Positive zooms
        /// <b>in</b> (toward the character). Set through <see cref="Forward"/>.
        /// </summary>
        public float ZoomRate;

        /// <summary>Closest the camera may zoom (compared squared, as 0.49).</summary>
        public const float MinZoomDistance = 0.7f;

        /// <summary>Furthest the camera may zoom (compared squared, as 625).</summary>
        public const float MaxZoomDistance = 25f;

        /// <summary>A zoom stops once less than this much distance is left to cover.</summary>
        public const float ZoomStopThreshold = 0.3f;

        /// <summary>Below this distance an inward zoom is not serviced at all.</summary>
        public const float ZoomNearGate = 0.8f;

        /// <summary>
        /// How much distance the pending zoom still has to cover. This is a <b>distance
        /// remaining</b>, not a speed: it is decremented by the distance the camera actually
        /// travelled each frame.
        /// </summary>
        public float ZoomPending { get; private set; }

        /// <summary>The camera's distance last frame.</summary>
        float _previousZoomDistance;

        /// <summary>
        /// A zoom can be requested before the first tick; without this an unseeded zero would be
        /// folded in as a huge bogus delta.
        /// </summary>
        bool _zoomDistanceSeeded;

        /// <summary>
        /// Despite the name this is the zoom control; it just stores the rate.
        /// </summary>
        public virtual void Forward(float rate) => ZoomRate = rate;

        /// <summary>Queue a zoom of <paramref name="delta"/> metres. Positive zooms in.</summary>
        public void RequestZoom(float delta) => ZoomPending += delta;

        /// <summary>Orbit only re-seats the goal when the camera is slower than this.</summary>
        public const float OrbitResyncSpeed = 0.02f;

        /// <summary>Zoom speed factor.</summary>
        public const float ZoomSpeed = 3f;

        /// <summary><see cref="CameraViewMode.Lock"/> clamps the zoomed distance to this floor.</summary>
        public const float ZoomDirectMinDistance = 0.78f;

        /// <summary><see cref="CameraViewMode.Lock"/> clamps the zoomed distance to this ceiling.</summary>
        public const float ZoomDirectMaxDistance = 25f;

        /// <summary>Above this much remaining, zoom moves proportionally rather than at a fixed rate.</summary>
        public const float ZoomProportionalThreshold = 1f;

        /// <summary>Lets <see cref="UpdateZoom"/> reset the pending zoom from a subclass.</summary>
        protected void ClearZoomPending() => ZoomPending = 0f;

        /// <summary>Lets a subclass spend down the pending zoom.</summary>
        protected void SpendZoomPending(float amount) => ZoomPending -= amount;

        /// <summary>
        /// Below this distance the fixed third-person camera starts blending its facing toward the
        /// character's own.
        /// </summary>
        public const float VetoBlendDistance = 0.9f;

        /// <summary>
        /// Which way the camera faces: straight at the look target, or the reference forward if it is
        /// sitting on top of it.
        ///
        /// The vetoes are not position constraints, despite the name. The body orientation update
        /// (orientation mode 3, the camera's) calls them, and they decide only the rotation.
        /// </summary>
        public virtual bool VetoForward(out Vec3 forward)
        {
            forward = GetLookTargetPos() - Position;

            if (forward.IsZero)
                forward = Vec3.ReferenceForward;
            else
                forward = forward / forward.Length;

            return true;
        }

        /// <summary>
        /// World up, orthogonalised against the forward, falling back to the reference forward when
        /// the camera looks straight up or down. Note it <b>discards</b> the surface normal the caller passed in: the camera is
        /// never banked by the ground.
        /// </summary>
        public virtual bool VetoUpAlignment(out Vec3 up)
        {
            up = Vec3.ReferenceUp;

            VetoForward(out Vec3 forward);
            if (forward.IsZero)
                return true;

            // Looking straight up or down leaves no usable up; swap in the reference forward.
            if (forward.X == 0f && forward.Z == 0f)
                up = Vec3.ReferenceForward;

            up -= forward * Vec3.Dot(up, forward);

            float len = up.Length;
            if (len != 0f)
                up = up / len;

            return true;
        }

        /// <summary>
        /// Adopt the camera's current position as its preferred one. <c>true</c> also adopts the distance, which is how a
        /// finished zoom is committed.
        /// </summary>
        public virtual void ForcedUpdate(bool recalcDistance) { }

        /// <summary>
        /// The zoom for every mode except <see cref="CameraViewMode.Lock"/>: it steers the zoom through
        /// <see cref="ZoomSteer"/>, folding the distance actually covered back into
        /// <see cref="ZoomPending"/> each frame. Lock moves the camera directly instead — see
        /// <see cref="CameraVehicleFixedThirdSim.UpdateZoom"/>.
        /// </summary>
        public virtual void UpdateZoom(float dt)
        {
            float distance = (GetLookTargetPos() - Position).Length;

            if (!_zoomDistanceSeeded)
            {
                _previousZoomDistance = distance;
                _zoomDistanceSeeded = true;
            }

            // An inward zoom is not serviced at all once the camera is inside the near gate.
            if (distance < ZoomNearGate && ZoomPending > 0f)
                return;

            if (ZoomPending != 0f)
            {
                bool wasPositive = ZoomPending > 0f;
                ZoomPending = (distance - _previousZoomDistance) + ZoomPending;

                if ((ZoomPending > 0f) == wasPositive
                    && Math.Abs(ZoomPending) >= ZoomStopThreshold)
                {
                    Forward(ZoomPending);
                }
                else
                {
                    ZoomPending = 0f;
                    Forward(0f);
                    Halt();
                    ForcedUpdate(true);
                }
            }

            _previousZoomDistance = distance;
        }

        /// <summary>
        /// Seeks a point placed along the camera-to-head direction, scaled by twice the rate — so a
        /// positive rate puts the goal past the character and the camera closes in, a negative one
        /// puts it behind the camera and it backs off.
        ///
        /// The limits are compared as <b>squared</b> distances, 0.49 and 625, i.e. 0.7 m and 25 m.
        /// </summary>
        public SteeringResult ZoomSteer(float rate, out Vec3 steer)
        {
            Vec3 lookTarget = GetLookTargetPos();
            Vec3 toLook = lookTarget - Position;
            float d2 = toLook.LengthSquared;

            bool canZoomIn = d2 > MinZoomDistance * MinZoomDistance && rate > 0f;
            bool canZoomOut = d2 < MaxZoomDistance * MaxZoomDistance && rate < 0f;

            if (!canZoomIn && !canZoomOut)
                return SteeringHalt(out steer);

            return SteeringSeek(lookTarget + toLook * (rate + rate), out steer);
        }

        /// <summary>
        /// The point the camera looks at and orbits: the target's position plus its local eye offset
        /// rotated into world space.
        /// </summary>
        public Vec3 GetLookTargetPos() => TargetPosition + TargetRotation * EyeTargetLocalPos;

        /// <summary>
        /// Measures how far the camera actually is from what it is looking at and adopts that as the
        /// follow distance, floored at <see cref="MinFollowDistance"/>.
        ///
        /// <b>This is not a per-frame tick, despite the name.</b> Call it only right after moving the
        /// camera by force (a teleport), to re-sync the stored distance to where the camera was put.
        /// Calling it every frame feeds the camera's current distance back into the goal that
        /// determines that distance, and the camera collapses onto the character.
        /// </summary>
        public virtual void ResyncFollowDistance()
        {
            float d = (GetLookTargetPos() - Position).Length;
            FollowDistance = d < MinFollowDistance ? MinFollowDistance : d;
        }

        /// <summary>
        /// Re-derives the speed and force limits every tick from how far behind the camera is, so it
        /// catches up after a teleport or a zone-in instead of crawling.
        ///
        /// <c>maxForce</c> is <c>mass * maxVel / 0.3</c> and <c>brakeDistance</c> is
        /// <c>maxVel * 0.3</c> — both fall out of "reach top speed in 0.3 s".
        /// </summary>
        /// <param name="baseSpeed">The configured camera speed.</param>
        public void UpdateMotionConstraints(float baseSpeed)
        {
            // The CAMERA's own distance from what it is looking at. Measuring from the character's
            // body instead yields the constant eye-offset length, which silently disables the
            // catch-up curve and lets the camera trail metres behind a walking character.
            float distance = (GetLookTargetPos() - Position).Length;

            if (distance > CatchUpDistance)
            {
                float boost = (float)Math.Sqrt(distance - CatchUpFalloff) + 1f;
                float boosted = boost * baseSpeed;
                baseSpeed = boosted <= MaxCatchUpSpeed ? boosted : MaxCatchUpSpeed;
            }

            if (baseSpeed < MinCatchUpSpeed)
                baseSpeed = MinCatchUpSpeed;

            MaxVel = baseSpeed;
            MaxForce = (Mass * baseSpeed) / ForceReachTime;
            SlowingDistance = (baseSpeed * baseSpeed * Mass) / MaxForce;   // == baseSpeed * 0.3
        }

        /// <summary>
        /// One frame of the camera: re-stamp mass, derive the base speed from the character's speed,
        /// re-derive the motion constraints, zoom, then integrate.
        /// </summary>
        /// <param name="characterMaxSpeed">
        /// The followed character's <b>maximum</b> speed, not its current one. The current speed makes
        /// the camera crawl at the 2 m/s floor whenever the character stands still, which shows up as
        /// an orbit that takes seconds to catch up with a mouse flick.
        /// </param>
        public virtual void Tick(float dt, float characterMaxSpeed)
        {
            Mass = 20f;

            float baseSpeed = characterMaxSpeed * BaseSpeedFromCharacterSpeed;
            if (baseSpeed > MaxBaseSpeed)
                baseSpeed = MaxBaseSpeed;

            UpdateMotionConstraints(baseSpeed);
            UpdateZoom(dt);
            Run(dt);
        }

        /// <summary>
        /// The obstacle-avoidance nudge from the camera's sensors. <b>Not implemented:</b> it stays
        /// zero ("no obstacle"), which makes Trail behave like a plain arrive.
        /// </summary>
        public Vec3 SensorSteerDir;

        /// <summary>Set when the sensors report the view blocked.</summary>
        public bool IsBlind;

        /// <summary>
        /// The direct-control throttle <see cref="DoDirectControl"/> reads. Zero unless a direct
        /// camera-fly control is active.
        /// </summary>
        public float DirectControlThrottle;

        /// <summary>
        /// When a direct throttle is set, drive the camera forward or back and skip the normal
        /// steering entirely.
        /// </summary>
        protected SteeringResult DoDirectControl(out Vec3 steer)
        {
            if (DirectControlThrottle == 0f)
            {
                steer = Vec3.Zero;
                return SteeringResult.None;
            }

            return DirectControlThrottle < 0f
                ? SteeringReverse(out steer)
                : SteeringForward(out steer);
        }

        /// <summary>
        /// The <b>Trail</b> brain (<see cref="CameraViewMode.Trail"/>), in priority order: a bound
        /// attractor wins, then a pending zoom, then direct control, then arrive at the desired point.
        ///
        /// The desired point is the look target pushed back along the current camera direction by
        /// the follow distance, raised by 0.4 when the camera is below the look target or the sensors
        /// report something within 0.4, plus the sensor nudge.
        /// </summary>
        protected override SteeringResult CalcSteering(out Vec3 force)
        {
            if (ZoomRate != 0f)
                return ZoomSteer(ZoomRate, out force);

            SteeringResult direct = DoDirectControl(out force);
            if (direct != SteeringResult.None)
                return direct;

            Vec3 lookTarget = GetLookTargetPos();
            Vec3 toCamera = Position - lookTarget;

            float len = toCamera.Length;
            Vec3 dir = len > 1e-6f
                ? toCamera / len
                : TargetRotation * Vec3.ReferenceForward * -1f;

            Vec3 desired = lookTarget + dir * FollowDistance;

            // Below the look target: lift the goal.
            if (Position.Y < lookTarget.Y)
                desired.Y += 0.4f;

            if (!IsBlind)
                desired += SensorSteerDir;

            return SteeringCamArrive(desired, out force, ArriveHaltRadius);
        }

        // ---- SteeringCamArrive ----------------------------------------------

        float _smoothedStep;
        bool _smoothedStepSeeded;

        /// <summary>
        /// <c>SteeringArrive</c> with two camera-specific additions:
        ///
        /// <para>A <b>hitch guard</b>: it keeps a running half-and-half average of the step size and
        /// halts outright if the current step is more than ten times it, so a loading spike parks
        /// the camera instead of slinging it. It measures the <b>sub-step</b>, not the frame, so with
        /// the camera's 0.05 s cap an ordinary frame-rate drop never trips it — only a collapse from
        /// a high baseline does.</para>
        ///
        /// <para>A <b>swing-around</b>: when the camera is closer to what it is looking at than to
        /// where it is trying to get, the target is over a metre away, and the two planar directions
        /// are nearly collinear (|sin| &lt; 0.4) and pointing the same way, it pushes the goal
        /// sideways so the camera arcs around the character rather than straight through them.</para>
        /// </summary>
        public SteeringResult SteeringCamArrive(Vec3 target, out Vec3 steer, float haltRadius)
        {
            float step = DeltaTimeNow;

            if (!_smoothedStepSeeded)
            {
                _smoothedStep = step;
                _smoothedStepSeeded = true;
            }

            if (_smoothedStep * HitchStepRatio < step)
                return SteeringHalt(out steer);

            _smoothedStep = _smoothedStep * 0.5f + step * 0.5f;

            Vec3 lookTarget = GetLookTargetPos();

            // Both compared flat — the swing-around is a planar decision.
            Vec3 toLook = lookTarget - Position;
            toLook.Y = 0f;
            Vec3 toTarget = target - Position;
            toTarget.Y = 0f;

            if (!toLook.IsZero && !toTarget.IsZero)
            {
                float lookLen = toLook.Length;
                float targetLen = toTarget.Length;

                Vec3 lookDir = toLook / lookLen;
                Vec3 targetDir = toTarget / targetLen;

                // Y of the cross product is the signed sine between the two planar directions.
                float sin = Vec3.Cross(lookDir, targetDir).Y;

                if (lookLen < targetLen
                    && targetLen > 1f
                    && Math.Abs(sin) < 0.4f
                    && Vec3.Dot(lookDir, targetDir) > 0f)
                {
                    // Perpendicular to the look direction, in the ground plane.
                    var perpendicular = new Vec3(toLook.Z, 0f, -toLook.X);
                    target = lookTarget + perpendicular * (SwingScale(sin) * 0.5f);
                }
            }

            return SteeringArrive(target, out steer, haltRadius);
        }

        /// <summary>
        /// The scale the swing-around applies to the perpendicular. <b>Not implemented:</b> it keeps
        /// the sign of the sine, which decides which way the camera arcs, and leaves the magnitude at
        /// one.
        /// </summary>
        protected virtual float SwingScale(float sin) => sin < 0f ? -1f : 1f;
    }
}

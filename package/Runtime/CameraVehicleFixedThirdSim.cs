using System;

namespace LostEden.Vehicles
{
    /// <summary>
    /// Third-person camera at a preferred offset from the look target (view modes 2 and 3).
    ///
    /// Occlusion (<see cref="RecalcOptimalPos"/>) is a binary search along the line from the head
    /// to the ideal spot for the furthest point that can still see the head, not a sphere sweep.
    /// </summary>
    public class CameraVehicleFixedThirdSim : CameraVehicleSim
    {
        /// <summary>
        /// The default preferred camera offset, used when no preference is stored. Stored normalised in
        /// <see cref="PreferredDirection"/> with its length becoming the follow distance: 4.743 m from
        /// the head, 18.4 degrees above the horizon, directly behind.
        /// </summary>
        public static readonly Vec3 DefaultPreferredOffset = new Vec3(0f, 1.5f, -4.5f);

        /// <summary>Binary-search bracket, low end — never closer than this fraction.</summary>
        public const float OcclusionSearchLow = 0.01f;

        /// <summary>Binary-search bracket, high end — never further than this fraction.</summary>
        public const float OcclusionSearchHigh = 0.95f;

        /// <summary>Iteration cap on the occlusion search.</summary>
        public const int OcclusionSearchMaxIterations = 20;

        /// <summary>The search stops once the bracket is this tight.</summary>
        public const float OcclusionSearchTolerance = 0.001f;

        /// <summary>
        /// A unit direction from the look target to where the camera would like to sit, in the
        /// target's local frame when <see cref="StayBehind"/> is on.
        /// </summary>
        public Vec3 PreferredDirection;

        /// <summary>Rotates the preferred direction into the target's frame. On by default.</summary>
        public bool StayBehind = true;

        /// <summary>
        /// <c>false</c> for <see cref="CameraViewMode.Rubber"/>, <c>true</c> for
        /// <see cref="CameraViewMode.Lock"/>. When set, <c>CalcSteering</c> returns
        /// <see cref="SteeringResult.None"/> and <see cref="DecideSnap"/> places the camera instead.
        /// </summary>
        public bool Frozen;

        /// <summary>The proportion of the remaining distance Lock closes per snap.</summary>
        public const float LockEaseFraction = 0.1f;

        /// <summary>Where the camera would sit with nothing in the way.</summary>
        public Vec3 OptimalPos { get; private set; }

        /// <summary>
        /// The occlusion-adjusted goal. Zero is the sentinel for "nothing in the way, use
        /// <see cref="OptimalPos"/>".
        /// </summary>
        public Vec3 AdjustedPos { get; private set; }

        /// <summary>How many iterations the last occlusion search used. Diagnostics only.</summary>
        public int LastOcclusionIterations { get; private set; }

        /// <summary>
        /// Can <paramref name="from"/> see <paramref name="to"/>? Backed by the world; the default
        /// says yes so the maths is testable on its own.
        /// </summary>
        public Func<Vec3, Vec3, bool> LineOfSight = (from, to) => true;

        /// <summary>
        /// Rubber steers, so the base tick is enough. Lock does not steer at all — it is re-placed
        /// every frame, so this drives <see cref="DecideSnap"/> instead.
        /// </summary>
        public override void Tick(float dt, float characterMaxSpeed)
        {
            base.Tick(dt, characterMaxSpeed);

            if (Frozen)
            {
                RecalcOptimalPos();
                DecideSnap();
            }
        }

        /// <summary>Applies the camera's factory settings and the default camera offset.</summary>
        public void ApplyDefaults()
        {
            ApplyDefaultSettings();
            SetPreferredOffset(DefaultPreferredOffset);
        }

        /// <summary>
        /// The offset's length becomes the follow distance and the offset itself is stored as a unit
        /// direction.
        /// </summary>
        public void SetPreferredOffset(Vec3 offset)
        {
            float len = offset.Length;
            FollowDistance = len;
            PreferredDirection = len == 0f ? Vec3.Zero : offset / len;
        }

        public Vec3 GetOptimalPos() => OptimalPos;

        /// <summary>
        /// The orbit primitive: "put the camera at this world position". It converts the position into
        /// a preferred direction (into the target's local frame when <see cref="StayBehind"/> is on),
        /// optionally adopts its length as the new follow distance, clears the occlusion sentinel and
        /// recomputes.
        ///
        /// The orbit driver ends every mouse drag with this, so it is the right entry point for camera
        /// input.
        /// </summary>
        /// <param name="recalcDistance">
        /// True adopts <c>|pos - lookTarget|</c> as the follow distance (a drag that also zooms);
        /// false keeps the current distance and only changes the direction, which is what an orbit
        /// uses.
        /// </param>
        public void UpdateHeadingToPos(Vec3 worldPosition, bool recalcDistance)
        {
            Vec3 lookTarget = GetLookTargetPos();
            Vec3 dir = worldPosition - lookTarget;

            if (StayBehind)
                dir = TargetRotation.Conjugate * dir;

            if (recalcDistance)
                FollowDistance = dir.Length;

            // Divides by the follow distance, not by the direction's own length — so with
            // recalcDistance false a position at a different range rescales the direction rather than
            // renormalising it.
            if (FollowDistance != 0f)
                dir = dir / FollowDistance;

            PreferredDirection = dir;
            AdjustedPos = Vec3.Zero;

            RecalcOptimalPos();
            DecideSnap();
        }

        /// <summary>
        /// Gated entirely on <see cref="Frozen"/>, so this does nothing in
        /// <see cref="CameraViewMode.Rubber"/> and is the entire positioning mechanism in
        /// <see cref="CameraViewMode.Lock"/>.
        ///
        /// Pulling in is instant; pushing out eases, closing a tenth of the gap per call
        /// (<c>newDistance = current * 0.9 + goal * 0.1</c>). The direction snaps either way.
        /// </summary>
        public void DecideSnap()
        {
            if (!Frozen)
                return;

            Vec3 lookTarget = GetLookTargetPos();
            float currentDistance = (Position - lookTarget).Length;

            Vec3 goal = AdjustedPos.IsZero ? OptimalPos : AdjustedPos;
            Vec3 toGoal = goal - lookTarget;
            float goalDistance = toGoal.Length;

            if (goalDistance <= currentDistance)
            {
                // Closer than we are: go straight there.
                SetRelPos(goal);
                return;
            }

            if (goalDistance <= 0f)
                return;

            float eased = currentDistance * (1f - LockEaseFraction) + goalDistance * LockEaseFraction;
            SetRelPos(lookTarget + (toGoal / goalDistance) * eased);
        }

        /// <summary>
        /// Places the ideal camera point, then — if a surface is bound and that point cannot see the
        /// head — binary-searches the segment from the head out to it for the furthest visible
        /// fraction.
        /// </summary>
        public void RecalcOptimalPos()
        {
            Vec3 dir = StayBehind ? TargetRotation * PreferredDirection : PreferredDirection;
            Vec3 lookTarget = GetLookTargetPos();

            OptimalPos = lookTarget + dir * FollowDistance;
            LastOcclusionIterations = 0;

            if (!HasSurface)
                return;

            if (LineOfSight(lookTarget, OptimalPos + dir * NearProbeOffset))
            {
                // The sentinel: nothing in the way, so CalcSteering falls back to OptimalPos.
                AdjustedPos = Vec3.Zero;
                return;
            }

            float lo = OcclusionSearchLow;
            float hi = OcclusionSearchHigh;
            int i = 0;
            Vec3 candidate;

            do
            {
                i++;
                float t = (lo + hi) * 0.5f;

                // Lerp from the head out towards the ideal spot.
                candidate = OptimalPos * t + lookTarget * (1f - t);
                AdjustedPos = candidate;

                if (LineOfSight(lookTarget, candidate + dir * NearProbeOffset))
                    lo = t;     // visible from here, try further out
                else
                    hi = t;     // blocked, pull in
            }
            while (i < OcclusionSearchMaxIterations && hi - lo > OcclusionSearchTolerance);

            LastOcclusionIterations = i;

            // Keeps the LAST candidate tested, which is not necessarily the last visible one.
        }

        /// <summary>
        /// Recompute the goal, then arrive at it with a 0.01 brake distance — unless a zoom is
        /// pending, or the camera is frozen.
        /// </summary>
        protected override SteeringResult CalcSteering(out Vec3 force)
        {
            RecalcOptimalPos();

            Vec3 target = AdjustedPos.IsZero ? OptimalPos : AdjustedPos;

            if (ZoomRate != 0f)
                return ZoomSteer(ZoomRate, out force);

            if (Frozen)
            {
                force = Vec3.Zero;
                return SteeringResult.None;
            }

            return SteeringCamArrive(target, out force, ArriveHaltRadius);
        }

        /// <summary>
        /// The zoom in <see cref="CameraViewMode.Lock"/>. Unlike the other modes it does <b>not</b>
        /// steer: it walks the camera's distance in directly and places it each frame, so zoom tracks
        /// the wheel rather than trailing behind it.
        ///
        /// Over a metre of remaining travel it moves proportionally (<c>dt * pending * 3</c>), under
        /// a metre at a flat <c>dt * 3</c>. It stops when the remainder drops below 0.3 m or flips
        /// sign, clamps the distance to 0.78-25 m, and — when zooming <i>out</i> into something
        /// solid — reverts to where the camera was and cancels.
        /// </summary>
        public override void UpdateZoom(float dt)
        {
            if (ZoomPending == 0f)
                return;

            // Recompute OptimalPos here rather than rely on last frame's CalcSteering having left a
            // fresh one, so the result does not depend on the order Tick calls things in.
            RecalcOptimalPos();

            Vec3 lookTarget = GetLookTargetPos();
            Vec3 offset = OptimalPos - lookTarget;
            float distance = offset.Length;
            if (distance == 0f)
            {
                ClearZoomPending();
                return;
            }

            Vec3 dir = offset / distance;
            bool wasPositive = ZoomPending > 0f;

            float move = Math.Abs(ZoomPending) >= ZoomProportionalThreshold
                ? dt * ZoomPending * ZoomSpeed
                : dt * ZoomSpeed * (wasPositive ? 1f : -1f);

            distance -= move;
            SpendZoomPending(move);

            if ((ZoomPending > 0f) != wasPositive || Math.Abs(ZoomPending) < ZoomStopThreshold)
                ClearZoomPending();

            if (distance < ZoomDirectMinDistance)
                distance = ZoomDirectMinDistance;
            else if (distance > ZoomDirectMaxDistance)
                distance = ZoomDirectMaxDistance;

            Vec3 intended = lookTarget + dir * distance;
            Vec3 before = Position;

            SetRelPos(intended);

            // Zooming out into geometry: put it back and give up rather than shove through.
            if (!wasPositive && (Position - intended).LengthSquared > 1e-6f)
            {
                SetRelPos(before);
                ClearZoomPending();
            }

            ResyncFollowDistance();
            ForcedUpdate(false);
        }

        /// <summary>
        /// Beyond 0.9 m this is just the base veto — look straight at the character. Closer in, it blends the facing toward the
        /// direction the <i>character</i> is facing, by <c>t = 2 * (0.9 - distance)</c>, reaching
        /// fully at 0.4 m. That is what stops the view spinning wildly when the camera is jammed
        /// against the head; the zoom floor of 0.78 m puts it inside this range at full zoom-in.
        /// </summary>
        public override bool VetoForward(out Vec3 forward)
        {
            float distance = (GetLookTargetPos() - Position).Length;

            if (distance > VetoBlendDistance)
                return base.VetoForward(out forward);

            base.VetoForward(out Vec3 toTarget);

            Vec3 characterFacing = TargetRotation * Vec3.ReferenceForward;
            float t = (VetoBlendDistance - distance) + (VetoBlendDistance - distance);

            if (t >= 1f)
            {
                forward = characterFacing;
                return true;
            }

            forward = toTarget * (1f - t) + characterFacing * t;

            float len = forward.Length;
            if (len != 0f)
                forward = forward / len;

            return true;
        }

        /// <summary>
        /// Stamps the camera's current position in as its preferred one. With <paramref name="recalcDistance"/>
        /// it also adopts the distance, which is how the driver commits a finished zoom.
        /// </summary>
        public override void ForcedUpdate(bool recalcDistance)
            => UpdateHeadingToPos(Position, recalcDistance);
    }
}

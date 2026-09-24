using System;
using N3Lite.Surfaces;

namespace N3Lite
{
    /// <summary>
    /// The integrator characters and the camera share: three steering channels in, a sub-stepped
    /// semi-implicit Euler step, and ground contact through an <see cref="ISurface"/>. Subclasses
    /// supply the steering.
    /// </summary>
    public class VehicleSim
    {
        /// <summary>m/s².</summary>
        public const float GravityAccel = -20f;

        /// <summary>Clamp on the gravity accumulator alone, not on speed.</summary>
        public const float MaxFallSpeed = 50f;

        /// <summary>A longer frame is dropped whole, with no integration at all.</summary>
        public const float MaxFrameTime = 4f;

        /// <summary>Below this speed the vehicle counts as stopped.</summary>
        public const float MovingSpeedEpsilon = 0.001f;

        /// <summary>A turn axis shorter than this is ignored.</summary>
        public const float TurnEpsilon = 0.0001f;

        /// <summary>
        /// The current sub-step, set at the top of each. One value shared by every vehicle, so
        /// vehicles must not be stepped on several threads at once.
        /// </summary>
        public static float DeltaTimeNow { get; private set; }

        /// <summary>Floored at 0.1.</summary>
        public float Mass
        {
            get => _mass;
            set => _mass = value <= 0f ? 0.1f : value;
        }
        float _mass = 50f;

        /// <summary>The steering force is truncated to this before integration.</summary>
        public float MaxForce = 2f;

        /// <summary>Velocity is truncated to this after integration.</summary>
        public float MaxVel = 2f;

        /// <summary>The distance over which <see cref="SteeringArrive"/> slows down — the arrive damping.</summary>
        public float SlowingDistance = 0.1f;

        /// <summary><see cref="SteeringArrive"/> stops outright inside this when the caller passes no radius.</summary>
        public float HaltRadius;

        /// <summary>The body half-extent; also how far the camera's occlusion probe is pushed out.</summary>
        public float NearProbeOffset = 0.01f;

        /// <summary>
        /// The sub-step ceiling. It bounds how far a long frame diverges; at 30 fps and above it never
        /// binds, so it does not make the result frame-rate independent.
        /// </summary>
        public float MaxSubStep = 0.4f;

        /// <summary>The camera's tighter sub-step: at least 20 Hz.</summary>
        public const float CameraSubStep = 0.05f;

        /// <summary>The vertical lift applied before probing.</summary>
        public const float ProbeLift = 0.4f;

        /// <summary>The ground tripod's radius.</summary>
        public const float ProbeRadius = 0.04f;

        /// <summary>Attempts the veto retry makes, backing off a tenth of the move each time.</summary>
        public const int VetoRetries = 9;

        /// <summary>The body half-extent floor.</summary>
        public const float BodyHalfExtent = 0.48f;

        /// <summary>Scales the movement length into the probe half-extent (2/√3). Not a slope limit.</summary>
        public const float HalfExtentPerMetre = 1.154700517654419f;

        /// <summary>A surface whose normal Y is below this is not walkable.</summary>
        public const float SlopeNormalY = 0.5f;

        /// <summary>The walkable limit when <see cref="RelaxSlopeGate"/> is set.</summary>
        public const float RelaxedSlopeNormalY = 0.001f;

        public const float StepHeight = 0.01f;

        /// <summary>Step height for collision profile 4, the camera's.</summary>
        public const float CameraStepHeight = 0.25f;

        public const float SweptHalfWidth = 0.4f;

        /// <summary>The swept move's iteration cap.</summary>
        public const int SweptIterations = 10;

        /// <summary>
        /// The sweep's reach, re-seated on every free move and every slide — a constant reach, not a
        /// budget.
        /// </summary>
        public const float LookAhead = 10f;

        /// <summary>The squared distance a missed probe contributes; beyond anything a 10 m ray reaches.</summary>
        public const float NoHitDistance = 10000f;

        static readonly Vec3[] ProbeDirections =
        {
            new Vec3(1f, 0f, 0f),
            new Vec3(-0.70710678f, 0f, 0.70710678f),
            new Vec3(-0.70710678f, 0f, -0.70710678f),
        };

        /// <summary>
        /// The wider second tripod, used in orientation mode 1: its longer baseline gives a steadier
        /// normal.
        /// </summary>
        static readonly Vec3[] SecondProbeDirections =
        {
            new Vec3(-1f, 0f, 0f),
            new Vec3(0.70710678f, 0f, 0.70710678f),
            new Vec3(0.70710678f, 0f, -0.70710678f),
        };

        public const float SecondProbeRadius = 0.2f;

        public const float SecondProbeReach = 0.8f;

        /// <summary>Weight on the new normal in the smoothing — per call, not per second.</summary>
        public const float NormalSmoothing = 0.1f;

        public Vec3 SmoothedSurfaceNormal = Vec3.ReferenceUp;

        public Vec3 SurfaceNormal = Vec3.ReferenceUp;

        /// <summary>4 is the camera's (taller step height); characters use 0.</summary>
        public int CollisionProfile;

        public bool RelaxSlopeGate;

        /// <summary>
        /// How the orientation update turns the body: 0 heading only, 1 aligned to the surface normal
        /// (characters), 3 the camera's through its vetoes. 2 is not implemented.
        /// </summary>
        public int OrientationMode;

        public Vec3 OrientationUp = Vec3.ReferenceUp;

        /// <summary>Orientation mode 1.</summary>
        public void UseSurfaceNormal() => OrientationMode = 1;

        public Vec3 Position;

        public Vec3 Velocity;

        /// <summary>
        /// The visible body rotation. Assigning it refreshes the cached forward
        /// <see cref="GetBodyForward"/> returns; without that a turn on the spot is undone next frame,
        /// because the orientation update rebuilds the rotation from that cached forward while stopped.
        /// </summary>
        public Quat BodyRotation
        {
            get => _bodyRotation;
            set
            {
                _bodyRotation = value;
                CacheBodyForward();
            }
        }

        Quat _bodyRotation = Quat.Identity;

        /// <summary>The force the longitudinal channel produced this step.</summary>
        public Vec3 SteerForce;

        /// <summary>The gravity accumulator, kept apart from <see cref="Velocity"/>.</summary>
        public float VerticalVelocity;

        /// <summary>
        /// <c>|Velocity|</c> as of the last force integration. Translation is gated on this, not on the
        /// vector — see <see cref="SetVel"/>.
        /// </summary>
        public float Speed { get; protected set; }

        public Vec3 PreviousPosition { get; protected set; }

        public bool FallingEnabled;

        /// <summary>
        /// When on and not airborne, vertical motion is pinned to zero every step. Characters run with it
        /// off and falling on: their ground contact comes entirely from
        /// <see cref="EnsureSurfaceAlignment"/>.
        /// </summary>
        public bool SurfaceHug = true;

        /// <summary>In the air right now; gates gravity.</summary>
        public bool Airborne;

        public bool IsMoving => Speed > MovingSpeedEpsilon;

        /// <summary><see cref="Run"/> returns at once when this is false.</summary>
        protected virtual bool IsRunEnabled() => true;

        /// <summary>The longitudinal channel. Only <see cref="SteeringResult.Force"/> is integrated.</summary>
        protected virtual SteeringResult CalcSteering(out Vec3 force)
        {
            force = Vec3.Zero;
            return SteeringResult.None;
        }

        /// <summary>
        /// The lateral channel: a velocity applied straight to position. Only
        /// <see cref="SteeringResult.Lateral"/> is honoured.
        /// </summary>
        protected virtual SteeringResult CalcLateralSteering(out Vec3 lateral)
        {
            lateral = Vec3.Zero;
            return SteeringResult.None;
        }

        /// <summary>
        /// The turn channel: an axis whose length is the angular rate. Only
        /// <see cref="SteeringResult.Turn"/> is honoured.
        /// </summary>
        protected virtual SteeringResult CalcTurnSteering(out Vec3 turn)
        {
            turn = Vec3.Zero;
            return SteeringResult.None;
        }

        /// <summary>
        /// The ground clamp and collision gate: veto retry, swept move, step clamp, tripod ground probe,
        /// slope gate, normal smoothing, orientation. Returns whether the position changed; false ends
        /// the sub-step loop. With no surface it returns true at once.
        /// </summary>
        protected virtual bool EnsureSurfaceAlignment(Vec3 previousPosition, bool force)
        {
            ISurface surface = GetSurface();
            if (surface == null)
                return true;

            Vec3 position = Position;

            // A float, not a flag: it scales the movement's Y, so a grounded sweep stays flat.
            float forced = (!FallingEnabled || Airborne || force) ? 1f : 0f;

            Vec3 tenth = (position - previousPosition) * 0.1f;
            for (int i = VetoRetries; i > 0; i--)
            {
                if (!surface.VetoPosition(ref position, this, previousPosition))
                    break;
                position = previousPosition + tenth * (float)i;
            }

            Vec3 delta = position - previousPosition;
            Vec3 lift = FallingEnabled ? new Vec3(0f, ProbeLift, 0f) : Vec3.Zero;

            // The probe works from the PREVIOUS position, lifted.
            Vec3 probe = previousPosition + new Vec3(0f, ProbeLift, 0f);
            float ceilingY = previousPosition.Y + ProbeLift;

            float movementLength = delta.Length;
            Vec3 horizontal = new Vec3(delta.X, 0f, delta.Z);
            float horizontalLength = horizontal.Length;

            if (movementLength > 0f)
            {
                if (force)
                {
                    probe += new Vec3(delta.X, delta.Y * forced, delta.Z);
                }
                else
                {
                    Vec3 swept = new Vec3(delta.X, delta.Y * forced, delta.Z);
                    Vec3 direction = swept;
                    if (direction.IsZero)
                        direction = new Vec3(0f, -1f, 0f);
                    else
                        direction = direction * (1f / direction.Length);

                    float totalBudget = horizontalLength < 1e-6f ? movementLength : 10000f;
                    if (forced != 0f)
                        totalBudget = movementLength;

                    float horizontalBudget = horizontalLength;
                    float slopeLimit = (FallingEnabled && !RelaxSlopeGate) ? 0.5f : -1f;

                    SweptMove(
                        ref probe, SweptHalfWidth, direction,
                        ref horizontalBudget, ref totalBudget,
                        surface, ref ceilingY,
                        FallingEnabled ? 3 : 1, SweptIterations, slopeLimit);
                }
            }

            ceilingY = force ? probe.Y + 1f : Math.Max(ceilingY, probe.Y);

            probe.Y -= ProbeLift;
            surface.CalculateClosestPoint(probe, out Vec3 closest, out _, this);

            float stepHeight = CollisionProfile == 4 ? CameraStepHeight : StepHeight;
            if (probe.Y < closest.Y + stepHeight)
                probe.Y = closest.Y + stepHeight;

            Vec3 high = new Vec3(probe.X, ceilingY, probe.Z) + lift;
            Vec3 low = new Vec3(probe.X, 0f, probe.Z);

            Vec3 h0 = ProbeOne(surface, high, low, ProbeDirections[0]);
            Vec3 h1 = ProbeOne(surface, high, low, ProbeDirections[1]);
            Vec3 h2 = ProbeOne(surface, high, low, ProbeDirections[2]);

            Vec3 normal = Vec3.Cross(h1 - h0, h2 - h0);
            if (normal.Y < 0f)
                normal = -normal;

            float normalLength = normal.Length;
            if (normalLength > 0f)
                normal = normal * (1f / normalLength);
            else
                normal = Vec3.ReferenceUp;

            float tripodMax = Math.Max(h0.Y, Math.Max(h1.Y, h2.Y));

            float halfExtent = BodyHalfExtent;
            if (FallingEnabled && !Airborne && !force)
            {
                halfExtent = movementLength * HalfExtentPerMetre + BodyHalfExtent;
                if (ceilingY < halfExtent)
                    halfExtent = ceilingY;
            }

            // Airborne unless at the ground AND on a walkable surface. A too-steep face leaves the body
            // airborne, so it slides down; climbing one is refused in the swept move.
            bool airborneNow = true;

            if (probe.Y - halfExtent <= tripodMax)
            {
                float groundRef = tripodMax;
                if (groundRef < closest.Y)
                    groundRef = closest.Y;

                bool descending = VerticalVelocity < 0.1f;

                if (descending && FallingEnabled)
                    probe.Y = groundRef + StepHeight;

                float gate = RelaxSlopeGate ? RelaxedSlopeNormalY : SlopeNormalY;
                if (normal.Y >= gate && descending)
                    airborneNow = false;
            }

            if (OrientationMode == 1)
            {
                Vec3 secondHigh = probe + new Vec3(0f, ProbeLift, 0f);
                Vec3 secondLow = probe - new Vec3(0f, ProbeLift, 0f);

                Vec3 s0 = ProbeOneAt(surface, secondHigh, secondLow, SecondProbeDirections[0]);
                Vec3 s1 = ProbeOneAt(surface, secondHigh, secondLow, SecondProbeDirections[1]);
                Vec3 s2 = ProbeOneAt(surface, secondHigh, secondLow, SecondProbeDirections[2]);

                Vec3 second = Vec3.Cross(s1 - s0, s2 - s0);
                if (second.Y < 0f)
                    second = -second;

                float secondLength = second.Length;
                normal = secondLength > 0f ? second * (1f / secondLength) : Vec3.ReferenceUp;
            }
            else
            {
                normal = Vec3.ReferenceUp;
            }

            // Smooth first, then square up anything steeper than the gate.
            Vec3 blended = SmoothedSurfaceNormal * (1f - NormalSmoothing) + normal * NormalSmoothing;
            float blendedLength = blended.Length;
            SmoothedSurfaceNormal = blendedLength > 0f ? blended * (1f / blendedLength) : Vec3.ReferenceUp;

            normal = SmoothedSurfaceNormal;
            if (normal.Y < SlopeNormalY)
                normal = Vec3.ReferenceUp;
            SurfaceNormal = normal;

            if (FallingEnabled)
            {
                if (airborneNow)
                    BeginFalling();
                else if (Airborne)
                    LandNow(probe.Y);
            }

            surface.VetoPosition(ref probe, this, previousPosition);
            Position = probe;

            bool moved = !Position.Equals(previousPosition);

            bool reoriented = UpdateOrientation(normal);
            return moved || reoriented;
        }

        /// <summary>
        /// Continuous collision with wall sliding. <paramref name="mode"/> above 1 adds the side and
        /// shoulder probes; <paramref name="slopeLimit"/> is 0.5 for a falling body, -1 (none) otherwise.
        ///
        /// A grounded body sweeps along a horizontal direction, which is why the steep-slope test fires
        /// for <c>move.y == 0</c> as well as upward moves.
        /// </summary>
        protected void SweptMove(
            ref Vec3 position,
            float halfWidth,
            Vec3 direction,
            ref float horizontalBudget,
            ref float totalBudget,
            ISurface surface,
            ref float ceilingY,
            int mode,
            int iterations,
            float slopeLimit)
        {
            Vec3 target = position + direction * LookAhead;

            while (iterations-- > 0)
            {
                Vec3 move = target - position;
                if (move.LengthSquared < 1e-08f)
                    return;
                move = move * (1f / move.Length);

                bool leftValid = false, rightValid = false;
                Vec3 leftHit = Vec3.Zero, rightHit = Vec3.Zero;
                Vec3 leftNormal = Vec3.Zero, rightNormal = Vec3.Zero;

                if (mode > 1)
                {
                    Vec3 side, other;
                    if (move.Y * move.Y < 0.999999f)
                    {
                        Vec3 across = Vec3.Cross(move, Vec3.ReferenceUp);
                        side = across * (halfWidth / across.Length);
                        other = -side;
                    }
                    else
                    {
                        side = new Vec3(halfWidth, 0f, 0f);
                        other = new Vec3(-halfWidth, -0f, -0f);
                    }

                    if (surface.GetLineIntersection(
                            position, position + side, out Vec3 sideHit, out _, true, this))
                        side = sideHit - position;
                    if (surface.GetLineIntersection(
                            position, position + other, out Vec3 otherHit, out _, true, this))
                        other = otherHit - position;

                    if (side.LengthSquared != other.LengthSquared)
                    {
                        Vec3 a = position + side;
                        Vec3 b = position + other;
                        position = (a + b) / 2f;
                        side = a - position;
                        other = b - position;

                        if (position.Y > ceilingY)
                            ceilingY = position.Y;
                    }

                    side = side * 0.5f;
                    other = other * 0.5f;

                    leftValid = ProbeShoulder(
                        surface, this, position, target, move, side, out leftHit, out leftNormal);
                    rightValid = ProbeShoulder(
                        surface, this, position, target, move, other, out rightHit, out rightNormal);
                }

                bool centreValid = surface.GetLineIntersection(
                    position, target, out Vec3 centreHit, out Vec3 centreNormal, true, this);

                if (!leftValid && !centreValid && !rightValid)
                {
                    if (position.Equals(target))
                        return;

                    Vec3 step = target - position;
                    float distance = step.Length;
                    step = step * ((distance - halfWidth) / distance);

                    if (Advance(ref position, step, ref horizontalBudget, ref totalBudget, ref ceilingY)
                        != AdvanceResult.Moved)
                        return;

                    target = position + direction * LookAhead;
                    continue;
                }

                float dLeft = leftValid ? (leftHit - position).LengthSquared : NoHitDistance;
                float dRight = rightValid ? (rightHit - position).LengthSquared : NoHitDistance;
                float dCentre = centreValid ? (centreHit - position).LengthSquared : NoHitDistance;

                Vec3 hit, normal;
                if (dRight > dLeft && dCentre > dLeft)
                {
                    hit = leftHit;
                    normal = leftNormal;
                }
                else if (dCentre > dRight)
                {
                    hit = rightHit;
                    normal = rightNormal;
                }
                else
                {
                    hit = centreHit;
                    normal = centreNormal;
                }

                // Too steep to walk: flatten the face to the vertical wall under it, or bounce straight back
                // when it has no horizontal component.
                float pushDistance = halfWidth;

                if (!(0f > move.Y) && slopeLimit > normal.Y)
                {
                    pushDistance = normal.Y * normal.Y * halfWidth + halfWidth;

                    bool flatten;
                    if (normal.Y <= -0.99f)
                        flatten = false;
                    else if (normal.LengthSquared <= 0.001f)
                        flatten = false;
                    else if (Math.Abs(normal.X) > 0.001f)
                        flatten = true;
                    else
                        flatten = Math.Abs(normal.Z) > 0.001f;

                    if (flatten)
                    {
                        Vec3 flat = new Vec3(normal.X, 0f, normal.Z);
                        normal = flat * (1f / flat.Length);
                    }
                    else
                    {
                        normal = -move;
                    }
                }

                Vec3 back = -move;
                float d = Vec3.Dot(back, normal);
                if (0f > d)
                    d = -d;

                float toHitSquared = (hit - position).LengthSquared;

                // A hit nearer than the push-out distance REFUSES the move rather than clamping to the hit —
                // that is what stops a body climbing a face it cannot walk up.
                Vec3 newPosition = position;
                if (d > 0f)
                {
                    float t = pushDistance / d;
                    if (!(toHitSquared < t * t))
                        newPosition = hit + back * t;
                }
                else if (!(halfWidth * halfWidth >= toHitSquared))
                {
                    newPosition = hit + back * halfWidth;
                }

                if (!newPosition.Equals(position))
                {
                    if (Advance(ref position, newPosition - position,
                                ref horizontalBudget, ref totalBudget, ref ceilingY)
                        == AdvanceResult.Exhausted)
                        return;
                }

                Vec3 toHit = position - hit;
                if (toHit.IsZero)
                    return;
                toHit = toHit * (1f / toHit.Length);

                float separation = (toHit - normal).LengthSquared;
                if (1e-05f > separation)
                    return;
                if (separation > 3.99999f)
                    return;
                if (normal.IsZero)
                    return;

                // The way the body was going, projected onto the face.
                Vec3 slide = Vec3.Cross(Vec3.Cross(toHit, normal), normal);
                slide = slide * (1f / slide.Length);

                if (!(Vec3.Dot(slide, direction) > 0f))
                    return;

                target = position + slide * LookAhead;

                // A perfectly vertical face is never climbed.
                if (normal.Y == 0f && position.Y < target.Y)
                    target.Y = position.Y;
            }
        }

        /// <summary>
        /// One shoulder's forward ray. The hit is projected back onto the movement line so all three
        /// candidates compare in one frame; a back face (the surface this shoulder already slides along)
        /// is discarded.
        /// </summary>
        static bool ProbeShoulder(
            ISurface surface, object locality, Vec3 position, Vec3 target, Vec3 move, Vec3 side,
            out Vec3 hit, out Vec3 normal)
        {
            hit = Vec3.Zero;
            normal = Vec3.Zero;

            if (side.IsZero)
                return false;

            if (!surface.GetLineIntersection(
                    position + side, target + side, out Vec3 rawHit, out normal, true, locality))
                return false;

            Vec3 unitSide = side * (1f / side.Length);
            if (Vec3.Dot(unitSide, normal) > 0f)
            {
                normal = Vec3.Zero;
                return false;
            }

            float denominator = Vec3.Dot(move, normal);
            if (denominator == 0f)
            {
                normal = Vec3.Zero;
                return false;
            }

            float t = Vec3.Dot(rawHit - position, normal) / denominator;
            hit = position + move * t;
            return true;
        }

        /// <summary>What <see cref="Advance"/> did.</summary>
        protected enum AdvanceResult
        {
            /// <summary>The whole step was taken and both budgets were charged.</summary>
            Moved,

            /// <summary>The step had no length.</summary>
            Degenerate,

            /// <summary>A budget ran out: the surviving fraction was taken, both budgets zeroed.</summary>
            Exhausted,
        }

        /// <summary>
        /// Charges <paramref name="step"/> against both budgets and moves the body. The horizontal arm
        /// never consults the total budget, so the total can go negative; only the other arm checks it.
        /// </summary>
        static AdvanceResult Advance(
            ref Vec3 position, Vec3 step,
            ref float horizontalBudget, ref float totalBudget, ref float ceilingY)
        {
            float horizontalSquared = new Vec3(step.X, 0f, step.Z).LengthSquared;
            float horizontal = horizontalSquared > 0f
                ? (float)Math.Sqrt(horizontalSquared)
                : horizontalSquared;

            if (horizontalBudget > 0f && horizontal > 0f)
            {
                if (horizontalBudget <= horizontal)
                {
                    position += step * (horizontalBudget / horizontal);
                    if (position.Y > ceilingY)
                        ceilingY = position.Y;
                    totalBudget = 0f;
                    horizontalBudget = 0f;
                    return AdvanceResult.Exhausted;
                }

                position += step;
                if (position.Y > ceilingY)
                    ceilingY = position.Y;
                horizontalBudget -= horizontal;
                totalBudget -= step.Length;
                return AdvanceResult.Moved;
            }

            float totalSquared = step.LengthSquared;
            if (!(0f < totalSquared))
                return AdvanceResult.Degenerate;

            float total = (float)Math.Sqrt(totalSquared);
            if (total <= 0f)
                return AdvanceResult.Degenerate;

            if (totalBudget <= total)
            {
                position += step * (totalBudget / total);
                if (position.Y > ceilingY)
                    ceilingY = position.Y;
                totalBudget = 0f;
                horizontalBudget = 0f;
                return AdvanceResult.Exhausted;
            }

            position += step;
            if (position.Y > ceilingY)
                ceilingY = position.Y;
            horizontalBudget -= horizontal;
            totalBudget -= total;
            return AdvanceResult.Moved;
        }

        /// <summary>
        /// Orientation mode 1: <c>LookRotation(forward, surfaceNormal)</c>, where forward is the velocity
        /// while moving and the cached forward while stopped. With <see cref="Direction"/> negative the
        /// body faces the negated forward, so a backpedalling character keeps facing the way it came;
        /// <see cref="SavedRotation"/> keeps the un-negated one.
        /// </summary>
        protected bool UpdateOrientation(Vec3 surfaceNormal)
        {
            if (OrientationMode != 1)
                return false;

            Vec3 forward = Speed == 0f ? GetBodyForward() : Velocity;

            OrientationUp = surfaceNormal;

            if (Direction < 0)
            {
                Vec3 back = -forward;
                BodyRotation = Quat.LookRotation(back, surfaceNormal);
                CacheBodyForward(back);
                SavedRotation = Quat.LookRotation(forward, surfaceNormal);
            }
            else
            {
                BodyRotation = Quat.LookRotation(forward, surfaceNormal);
                CacheBodyForward(forward);
                SavedRotation = BodyRotation;
            }

            return true;
        }

        /// <summary>
        /// Negative when the body travels backwards. Set it through <see cref="SetDirection"/>; changing
        /// it halts the vehicle.
        /// </summary>
        public int Direction { get; private set; } = 1;

        /// <summary>
        /// Pair it with every longitudinal drive change: forward is <c>SetDirection(1)</c> then drive 1;
        /// backward is drive -1 then <c>SetDirection(-1)</c>; release is drive 0, <c>Halt()</c>,
        /// <c>SetDirection(1)</c>. Without the -1 the body turns to face its backward velocity and
        /// oscillates on the spot.
        /// </summary>
        public void SetDirection(int direction)
        {
            if (direction != Direction)
            {
                SavedRotation = BodyRotation;
                HaltIfMoving();
            }

            Direction = direction;
        }

        /// <summary>
        /// Turns the body and re-aims the velocity along the new facing, keeping its magnitude and
        /// multiplying by <see cref="Direction"/>. The only correct way to set the heading of a moving
        /// vehicle: assigning <see cref="BodyRotation"/> alone is undone by orientation mode 1, which
        /// rebuilds the rotation from the velocity.
        /// </summary>
        public void SetRelRot(Quat rotation)
        {
            float speed = Velocity.Length;

            BodyRotation = rotation;
            SavedRotation = rotation;
            CacheBodyForward();

            Velocity = GetBodyForward() * speed * Direction;
        }

        /// <summary>Reversing stops the body dead first.</summary>
        void HaltIfMoving()
        {
            if (Speed == 0f)
                return;

            Speed = 0f;
            Velocity = Vec3.Zero;
            SavedRotation = BodyRotation;
            CacheBodyForward();
            OnHalt();
        }

        /// <summary>The un-negated facing: the way the body is actually travelling. Written, never read here.</summary>
        public Quat SavedRotation = Quat.Identity;

        Vec3 _cachedForward = Vec3.ReferenceForward;

        /// <summary>Caches the vector as is, without normalising.</summary>
        void CacheBodyForward(Vec3 forward) => _cachedForward = forward;

        /// <summary>Rebuilds the cache from the body rotation.</summary>
        void CacheBodyForward() => _cachedForward = BodyRotation * Vec3.ReferenceForward;

        /// <summary>
        /// A leg of the second tripod: near end offset by <see cref="SecondProbeRadius"/>, far end by
        /// <see cref="SecondProbeReach"/>. A miss returns the far end.
        /// </summary>
        Vec3 ProbeOneAt(ISurface surface, Vec3 high, Vec3 low, Vec3 direction)
        {
            Vec3 from = high + direction * SecondProbeRadius;
            Vec3 to = low + direction * SecondProbeReach;

            if (surface.GetLineIntersection(from, to, out Vec3 hit, out _, true, this))
                return hit;

            return to;
        }

        /// <summary>A leg of the ground tripod. A miss returns the low point.</summary>
        Vec3 ProbeOne(ISurface surface, Vec3 high, Vec3 low, Vec3 direction)
        {
            Vec3 offset = direction * ProbeRadius;
            Vec3 from = high + offset;
            Vec3 to = low + offset;

            if (surface.GetLineIntersection(from, to, out Vec3 hit, out _, true, this))
                return hit;

            return to;
        }

        /// <summary>
        /// Zeroes vertical motion, recomputes <see cref="Speed"/> from the velocity (translation is gated
        /// on it), notifies, then clears <see cref="Airborne"/> — in that order.
        /// </summary>
        public void LandNow(float height)
        {
            Velocity.Y = 0f;
            VerticalVelocity = 0f;
            Speed = Velocity.Length;
            OnLanded(height);
            Airborne = false;
        }

        /// <summary>What this vehicle collides against. Null makes <see cref="EnsureSurfaceAlignment"/> a no-op.</summary>
        protected virtual ISurface GetSurface() => null;

        /// <summary>Called when a blocked move gives up.</summary>
        protected virtual void OnHalt() { }

        /// <summary>Zeroes the motion state.</summary>
        public virtual void Halt()
        {
            Velocity = Vec3.Zero;
            SteerForce = Vec3.Zero;
            Speed = 0f;
        }

        /// <summary>
        /// Two branches: when falling is enabled it clears the flag and lands; when already disabled it
        /// zeroes vertical motion. Does not touch <see cref="SurfaceHug"/>.
        /// </summary>
        public void DisableFalling()
        {
            if (FallingEnabled)
            {
                FallingEnabled = false;
                LandNow(Position.Y);

                if (InLiquid)
                {
                    OnLeaveLiquid();
                    InLiquid = false;
                }

                return;
            }

            Velocity.Y = 0f;
            VerticalVelocity = 0f;
            Speed = Velocity.Length;
            Airborne = false;
        }

        /// <summary>
        /// Sets the flag, lands at the current height, then begins falling again. A no-op when already
        /// enabled.
        /// </summary>
        public void EnableFalling()
        {
            if (FallingEnabled)
                return;

            FallingEnabled = true;
            LandNow(Position.Y);
            BeginFalling();
        }

        /// <summary>Go airborne, once.</summary>
        public void BeginFalling()
        {
            if (Airborne)
                return;

            Airborne = true;
            OnBeginFall();
        }

        /// <summary>
        /// An impulse, accepted only when it is purely vertical and the body is on the ground; anything
        /// else is dropped whole. Adds <c>y / mass</c> to the gravity accumulator and begins falling.
        /// </summary>
        public void Impact(Vec3 impulse)
        {
            if (Airborne)
                return;
            if (!(impulse.X == 0f) || !(impulse.Z == 0f))
                return;

            VerticalVelocity += 1f / Mass * impulse.Y;
            BeginFalling();
        }

        public bool InLiquid;

        protected virtual void OnBeginFall() { }

        protected virtual void OnLanded(float height) { }

        protected virtual void OnLeaveLiquid() { }

        public void DisableSurfaceHug() => SurfaceHug = false;

        public void EnableSurfaceHug() => SurfaceHug = true;

        /// <summary>
        /// Sets the velocity <b>and</b> <see cref="Speed"/>. Assigning <see cref="Velocity"/> alone
        /// leaves the stored speed stale.
        /// </summary>
        public void SetVel(Vec3 velocity)
        {
            Velocity = velocity;
            Speed = velocity.Length;
        }

        /// <summary>
        /// Puts the vehicle at a position now, with collision, bypassing steering: how anything that must
        /// track input one-to-one (mouse-look orbit) moves a vehicle.
        /// </summary>
        public void SetRelPos(Vec3 position)
        {
            if (position.Equals(Position))
                return;

            Vec3 previous = Position;
            Position = position;
            PreviousPosition = position;
            EnsureSurfaceAlignment(previous, false);
        }

        /// <summary>
        /// One frame; returns whether the vehicle moved. <b>Not implemented:</b> pending relative moves
        /// and functional paths — only the physics path runs.
        /// </summary>
        public bool Run(float dt)
        {
            if (!IsRunEnabled())
                return false;

            return Step(dt);
        }

        /// <summary>
        /// The sub-stepping loop: steps of at most <see cref="MaxSubStep"/> until <paramref name="dt"/>
        /// is consumed.
        /// </summary>
        protected bool Step(float dt)
        {
            if (dt > MaxFrameTime || dt <= 0f)
                return false;

            bool moved = false;
            float elapsed = 0f;

            do
            {
                float step = dt - elapsed;
                if (step > MaxSubStep)
                    step = MaxSubStep;

                DeltaTimeNow = step;

                Vec3 stepStartPosition = Position;

                // 1. Longitudinal steering.
                SteeringResult longitudinal = CalcSteering(out Vec3 force);
                SteerForce = force;
                if (longitudinal == SteeringResult.Halt)
                {
                    SteerForce = Vec3.Zero;
                    Halt();
                }
                else if (longitudinal != SteeringResult.Force)
                {
                    SteerForce = Vec3.Zero;
                }

                if (Airborne)
                {
                    VerticalVelocity += GravityAccel * step;
                }
                else if (SurfaceHug)
                {
                    VerticalVelocity = 0f;
                    Velocity.Y = 0f;
                }

                if (longitudinal == SteeringResult.Force || Airborne)
                {
                    Vec3 f = SteerForce;
                    Vec3.Truncate(ref f, MaxForce);

                    Velocity += (f * step) / Mass;

                    if (VerticalVelocity > MaxFallSpeed)
                        VerticalVelocity = MaxFallSpeed;
                    else if (VerticalVelocity < -MaxFallSpeed)
                        VerticalVelocity = -MaxFallSpeed;

                    Vec3 v = Velocity;
                    Speed = Vec3.Truncate(ref v, MaxVel);
                    Velocity = v;
                }

                // 4. Lateral steering redirects, never adds speed.
                bool moving = IsMoving;
                Vec3 velocityThisStep = Velocity;

                SteeringResult lateralResult = CalcLateralSteering(out Vec3 lateral);
                if (lateralResult == SteeringResult.Halt)
                {
                    lateral = Vec3.Zero;
                }
                else if (lateralResult == SteeringResult.Lateral)
                {
                    if (!moving)
                    {
                        Vec3.Truncate(ref lateral, MaxVel);
                    }
                    else
                    {
                        float combined = (velocityThisStep + lateral).Length;
                        if (combined > 0f)
                        {
                            float k = Speed / combined;
                            velocityThisStep *= k;
                            lateral *= k;
                        }
                    }

                    Position += lateral * step;
                    moved = true;
                }

                if (moving || VerticalVelocity != 0f)
                {
                    Position += velocityThisStep * step;
                    Position.Y += VerticalVelocity * step;
                    moved = true;
                }

                // 6. Turn: rotates the body while standing still and the velocity while moving, so a moving
                // vehicle curves its path rather than sliding sideways.
                SteeringResult turnResult = CalcTurnSteering(out Vec3 turn);
                if (turnResult == SteeringResult.Halt)
                {
                    turn = Vec3.Zero;
                }
                else if (turnResult == SteeringResult.Turn)
                {
                    float rate = turn.Length;
                    if (rate > TurnEpsilon)
                    {
                        Vec3 axis = turn / rate;
                        float angle = rate * step;
                        Quat rotation = Quat.FromAxisAngle(axis, angle);

                        if (Velocity.X == 0f && Velocity.Z == 0f)
                            BodyRotation = (rotation * BodyRotation).Normalized;
                        else
                            Velocity = rotation * Velocity;
                    }

                    moved = true;
                }

                PreviousPosition = stepStartPosition;

                if (!EnsureSurfaceAlignment(stepStartPosition, false))
                    break;

                elapsed += step;
            }
            while (elapsed < dt);

            return moved;
        }

        /// <summary>Asks the integrator to stop this channel.</summary>
        public SteeringResult SteeringHalt(out Vec3 steer)
        {
            steer = Vec3.Zero;
            return SteeringResult.Halt;
        }

        /// <summary>
        /// Full <see cref="MaxForce"/> along the body's forward, so the integrator's clamp is exactly
        /// saturated.
        /// </summary>
        public SteeringResult SteeringForward(out Vec3 steer)
        {
            steer = GetBodyForward() * MaxForce;
            return SteeringResult.Force;
        }

        public SteeringResult SteeringReverse(out Vec3 steer)
        {
            steer = GetBodyForward() * -MaxForce;
            return SteeringResult.Force;
        }

        /// <summary>The cached forward, rebuilt only while the rotation is still the zero quaternion.</summary>
        public Vec3 GetBodyForward()
        {
            if (BodyRotation.X == 0f && BodyRotation.Y == 0f && BodyRotation.Z == 0f && BodyRotation.W == 0f)
                CacheBodyForward();

            return _cachedForward;
        }

        /// <summary>Flat out at the target: no slow-down, and scaled by <see cref="MaxForce"/> rather than mass.</summary>
        public SteeringResult SteeringSeek(Vec3 target, out Vec3 steer)
        {
            steer = Vec3.Zero;

            Vec3 toTarget = target - Position;
            float len = toTarget.Length;
            if (len == 0f)
                return SteeringResult.None;

            Vec3 desired = (toTarget / len) * MaxVel;
            steer = (desired - Velocity) * MaxForce;
            return SteeringResult.Force;
        }

        /// <summary>
        /// <see cref="SteeringArrive"/>, but halts once the body has passed the target: "target from
        /// here" and "target from last position" point opposite ways (XZ only). That is what stops an NPC
        /// orbiting a waypoint it cannot quite land on.
        /// </summary>
        public SteeringResult SteeringDirArrive(Vec3 target, out Vec3 steer)
        {
            float toHereX = target.X - Position.X;
            float toHereZ = target.Z - Position.Z;
            float toPrevX = target.X - PreviousPosition.X;
            float toPrevZ = target.Z - PreviousPosition.Z;

            float dot = toPrevZ * toHereZ + toPrevX * toHereX;
            if (dot < 0f)
                return SteeringHalt(out steer);

            return SteeringArrive(target, out steer, DirArriveBrakeDistance);
        }

        public const float DirArriveBrakeDistance = 0.2f;

        /// <summary>
        /// Slows inside <see cref="SlowingDistance"/> and halts inside <paramref name="haltRadius"/> (0
        /// means <see cref="HaltRadius"/>). The <c>* Mass * 4</c> aims to reach the desired velocity in
        /// 0.25 s; <see cref="MaxForce"/> is what actually limits the response.
        /// </summary>
        public SteeringResult SteeringArrive(Vec3 target, out Vec3 steer, float haltRadius = 0f)
        {
            if (haltRadius == 0f)
                haltRadius = HaltRadius;

            Vec3 toTarget = target - Position;
            float d2 = toTarget.LengthSquared;

            // Squared distances: 0.01 is 0.1 m.
            if (d2 < haltRadius * haltRadius || d2 < 0.01f)
                return SteeringHalt(out steer);

            float d = (float)Math.Sqrt(d2);

            float speed = (d / SlowingDistance) * MaxVel;
            if (speed > MaxVel)
                speed = MaxVel;

            Vec3 desired = toTarget * (speed / d);
            steer = (desired - Velocity) * (Mass * 4f);

            return steer.Length <= 1e7f ? SteeringResult.Force : SteeringResult.None;
        }
    }
}

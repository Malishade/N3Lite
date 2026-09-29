using System;
using System.Collections.Generic;
using N3Lite.Surfaces;

namespace N3Lite
{
    /// <summary>
    /// The vehicle an NPC moves with.
    ///
    /// <para>
    /// It is <b>not</b> a variant of the player vehicle. Two of the three steering channels do
    /// nothing: an NPC has <b>no strafe and no turn</b>. Its one channel walks a follow queue of
    /// waypoints, or chases a <see cref="PathGuide"/> along a <see cref="N3Lite.Path"/>. So the four
    /// input axes belong to the player alone.
    /// </para>
    ///
    /// <para>
    /// It still inherits everything in <see cref="CharVehicleSim"/> and <see cref="VehicleSim"/> —
    /// gravity, the ground clamp, collision with static geometry, the orientation update, the speed
    /// curve.
    /// </para>
    ///
    /// <para>
    /// Both a client and a server run it. It takes plain values; which message carries them is each
    /// side's own business.
    /// </para>
    /// </summary>
    public class NpcVehicleSim : CharVehicleSim
    {
        /// <summary>
        /// Squared XZ distance to the last waypoint under which a halt counts as the end of the
        /// path: 2 m.
        /// </summary>
        public const float PathEndRadiusSquared = 4f;

        /// <summary>
        /// The waypoints the guide slides along. Empty means "stand still": the longitudinal channel
        /// halts rather than returning <see cref="SteeringResult.None"/>.
        /// </summary>
        public Path Path { get; } = new Path();

        /// <summary>The point that slides along <see cref="Path"/>, which the NPC steers at.</summary>
        public PathGuide Guide { get; } = new PathGuide();

        /// <summary>
        /// How close the NPC gets to a follow waypoint before dropping it for the next: 1 m. It is
        /// also why an NPC stops about this far short of its last point.
        /// </summary>
        public float FollowDistance = 1f;

        readonly List<Vec3> _followQueue = new List<Vec3>();

        /// <summary>
        /// Set by <see cref="SetFollowPath"/>. While set the NPC walks <see cref="FollowQueue"/> and
        /// ignores <see cref="Path"/>; with the queue empty it halts where it is.
        /// </summary>
        public bool HasFollowTarget { get; private set; }

        /// <summary>The waypoints still to reach; the NPC steers at the first.</summary>
        public IReadOnlyList<Vec3> FollowQueue => _followQueue;

        /// <summary>
        /// The owner considers the NPC to be moving forward or back. Such an NPC keeps being stepped
        /// even with nothing to follow; see <see cref="IsRunEnabled"/>.
        /// </summary>
        public bool ForwardActive { get; set; }

        /// <summary>
        /// False holds a following NPC where it is. Whether an NPC may follow is the game's rule, so
        /// the owner sets this.
        /// </summary>
        public bool FollowEnabled { get; set; } = true;

        /// <summary>
        /// An NPC with nothing to follow is still marked <see cref="ForwardActive"/>, and should stop.
        /// Raised from <see cref="AdvanceGuide"/> every tick until the owner clears
        /// <see cref="ForwardActive"/> in response.
        /// </summary>
        public event Action StopRequested;

        /// <summary>
        /// Follows <paramref name="waypoints"/> in order. The Y is kept: unlike
        /// <see cref="N3Lite.Path"/>, the queue is not flattened.
        ///
        /// <para>
        /// With <paramref name="snapToStart"/> the first point is where the NPC is now: the body is
        /// snapped there (see <see cref="SnapTo"/>) and the queue is the points after it, or the start
        /// itself when nothing follows it. Without it the list is waypoints only.
        /// </para>
        /// </summary>
        public void SetFollowPath(IReadOnlyList<Vec3> waypoints, bool snapToStart = false)
        {
            HasFollowTarget = true;
            _followQueue.Clear();

            if (waypoints == null || waypoints.Count == 0)
                return;

            int first = 0;
            if (snapToStart)
            {
                SnapTo(waypoints[0]);

                // The start stays the first waypoint only when nothing follows it.
                if (waypoints.Count > 1)
                    first = 1;
            }

            for (int i = first; i < waypoints.Count; i++)
                _followQueue.Add(waypoints[i]);
        }

        /// <summary>How far below the start point the snap looks for ground.</summary>
        public const float SnapProbeDepth = 1.5f;

        /// <summary>
        /// Places the body at <paramref name="start"/>, dropped onto the ground when there is some
        /// within <see cref="SnapProbeDepth"/> below it, with no collision: the ray goes through the
        /// vehicle's own surface, and the position and previous position are both set.
        /// </summary>
        public void SnapTo(Vec3 start)
        {
            Vec3 at = start;
            ISurface surface = GetSurface();
            if (surface != null
                && surface.GetLineIntersection(
                    start, start - new Vec3(0f, SnapProbeDepth, 0f), out Vec3 hit, out _, false, null))
                at = hit;

            Position = at;
            PreviousPosition = at;
        }

        /// <summary>Stops following.</summary>
        public void ClearFollowTarget()
        {
            HasFollowTarget = false;
            _followQueue.Clear();
        }

        /// <summary>
        /// The follow point to steer at: drop every waypoint already within
        /// <see cref="FollowDistance"/> in XZ, then the first one left, or the body's
        /// own position when none is — which halts it.
        ///
        /// <para>
        /// With <see cref="FollowEnabled"/> off the answer is always the body's own position, before
        /// the queue is looked at.
        /// </para>
        /// </summary>
        Vec3 FollowSteerTarget()
        {
            if (!FollowEnabled)
                return Position;

            float reach = FollowDistance * FollowDistance;

            while (_followQueue.Count > 0)
            {
                float dx = _followQueue[0].X - Position.X;
                float dz = _followQueue[0].Z - Position.Z;
                if (!(dx * dx + dz * dz < reach))
                    return _followQueue[0];
                _followQueue.RemoveAt(0);
            }

            return Position;
        }

        /// <summary>
        /// Puts the guide back at the start of the current path at the body's top speed. The vehicle
        /// never calls this itself; whoever sets the path does.
        /// </summary>
        public void RestartPath()
        {
            Guide.RestartGuide(Path, MaxVel, 0f);
        }

        /// <summary>
        /// Advances the guide. It belongs to the AI tick, not to <c>Run</c>, so the caller drives it —
        /// once per frame with the frame's <c>dt</c>. Only a non-empty path advances, and the guide
        /// keeps the speed it was restarted with. With no path, no follow target and
        /// <see cref="ForwardActive"/> set it raises <see cref="StopRequested"/>.
        /// </summary>
        public void AdvanceGuide(float dt)
        {
            if (!Path.Empty)
                Guide.UpdateAddTime(dt);
            else if (!HasFollowTarget && ForwardActive)
                StopRequested?.Invoke();
        }

        /// <summary>
        /// Puts an NPC mid-walk: the velocity as is, then the path, and the guide restarted at
        /// <paramref name="guideTime"/> seconds along it at the current top speed. Set the movement
        /// profile first, so the top speed is the right one. <see cref="GetMotionState"/> reads the
        /// same values back, so one side can hand an NPC's motion to another.
        ///
        /// <para>
        /// A moving velocity is taken whole: an NPC already walking does not ramp up again. With no
        /// waypoints the guide time is ignored.
        /// </para>
        /// </summary>
        public void SetMotionState(Vec3 velocity, IReadOnlyList<Vec3> waypoints, float guideTime)
        {
            SetVel(velocity);

            Path.Clear();
            if (waypoints != null)
            {
                for (int i = 0; i < waypoints.Count; i++)
                    Path.AddWaypoint(waypoints[i]);
            }

            // A guide time means nothing without a path.
            if (waypoints == null || waypoints.Count == 0)
                guideTime = 0f;

            Guide.RestartGuide(Path, MaxVel, guideTime);
        }

        /// <summary>
        /// The same motion read back out: the velocity, the path's waypoints and the guide's time. The
        /// waypoints' Y is always 0, as <see cref="N3Lite.Path"/> flattens them.
        /// </summary>
        public void GetMotionState(out Vec3 velocity, List<Vec3> waypoints, out float guideTime)
        {
            velocity = Velocity;
            guideTime = Guide.Time;

            waypoints.Clear();
            for (int i = 0; i < Path.Size; i++)
                waypoints.Add(Path.GetWaypoint(i));
        }

        /// <summary>
        /// An NPC's vehicle is stepped only while it has something to do: a path, a follow target,
        /// <see cref="ForwardActive"/>, or a fall. An idle NPC on the ground is not stepped at all.
        /// </summary>
        protected override bool IsRunEnabled()
            => !Path.Empty || HasFollowTarget || ForwardActive || Airborne;

        /// <summary>
        /// A following NPC stops as any character does.
        /// On a path, a halt only ends the path when the path is empty or the body is within 2 m (XZ)
        /// of its last waypoint; any other halt leaves the path, and the steering picks it up again.
        /// </summary>
        protected override void OnHalt()
        {
            if (HasFollowTarget)
            {
                RaiseHalted();
                return;
            }

            bool ended = Path.Empty;
            if (!ended)
            {
                Vec3 last = Path.GetWaypoint(Path.Size - 1);
                float dx = last.X - Position.X;
                float dz = last.Z - Position.Z;
                ended = dx * dx + dz * dz < PathEndRadiusSquared;
            }

            if (!ended)
                return;

            Path.Clear();
            Guide.RestartGuide(Path, MaxVel, 0f);
            RaiseHalted();
        }

        /// <summary>
        /// The NPC's only steering channel: halt, walk the follow queue, or arrive at
        /// the path guide.
        ///
        /// <para>
        /// On a path the target's <b>Y is replaced by the body's own</b>, which keeps an NPC from
        /// steering up or down at a waypoint. Paths are flat anyway — <see cref="N3Lite.Path.AddWaypoint"/>
        /// discards Y.
        /// </para>
        /// </summary>
        protected override SteeringResult CalcSteering(out Vec3 force)
        {
            force = Vec3.Zero;

            // A locked drive brakes an NPC to a halt; a player's body just gets no force.
            if (DriveLocked)
                return SteeringHalt(out force);

            if (HasFollowTarget)
                return SteeringDirArrive(FollowSteerTarget(), out force);

            if (Path.Empty)                                          // halt, not None
                return SteeringHalt(out force);

            Vec3 target;
            if (Path.Size > 1)
                target = Guide.GuidePos;
            else if (Path.Size == 1)
                target = Path.GetWaypoint(0);
            else
                return SteeringHalt(out force);

            target.Y = Position.Y;
            return SteeringDirArrive(target, out force);
        }

        /// <summary>NPCs do not strafe.</summary>
        protected override SteeringResult CalcLateralSteering(out Vec3 lateral)
        {
            lateral = Vec3.Zero;
            return SteeringResult.None;
        }

        /// <summary>
        /// NPCs do not turn on the spot; their facing comes from the orientation update following the
        /// velocity.
        /// </summary>
        protected override SteeringResult CalcTurnSteering(out Vec3 turn)
        {
            turn = Vec3.Zero;
            return SteeringResult.None;
        }
    }
}

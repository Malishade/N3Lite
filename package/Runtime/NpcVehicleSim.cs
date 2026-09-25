using System;
using System.Collections.Generic;
using N3Lite.Surfaces;

namespace N3Lite
{
    /// <summary>
    /// The vehicle an NPC moves with (<c>NPCVehicle_t</c>).
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
    /// gravity, the ground clamp, statel collision, the orientation update, the speed curve.
    /// </para>
    ///
    /// <para>
    /// Both a client and a server run it. It takes plain values; which message carries them is each
    /// side's own business.
    /// </para>
    /// </summary>
    public class NpcVehicleSim : CharVehicleSim
    {
        /// <summary>Stock's follow queue is a fixed array of 30 points (<c>+0x190</c>).</summary>
        public const int FollowQueueCapacity = 30;

        /// <summary>
        /// Squared XZ distance to the last waypoint under which a halt counts as the end of the
        /// path: 2 m (<c>10161ad0</c>, read in <c>10071149</c>).
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
        /// How close the NPC gets to a follow waypoint before dropping it for the next: 1 m
        /// (<c>+0x3a4</c>, set in the constructor <c>10070cfe</c>). It is also why an NPC stops about
        /// this far short of its last point.
        /// </summary>
        public float FollowDistance = 1f;

        readonly List<Vec3> _followQueue = new List<Vec3>(FollowQueueCapacity);

        /// <summary>
        /// Set by <see cref="SetFollowPath"/>. While set the NPC walks <see cref="FollowQueue"/> and
        /// ignores <see cref="Path"/>; with the queue empty it halts where it is.
        /// </summary>
        public bool HasFollowTarget { get; private set; }

        /// <summary>The waypoints still to reach; the NPC steers at the first.</summary>
        public IReadOnlyList<Vec3> FollowQueue => _followQueue;

        /// <summary>
        /// The movement status's forward state is "moving". Stock keeps stepping such an NPC even
        /// with nothing to follow; see <see cref="IsRunEnabled"/>.
        /// </summary>
        public bool ForwardActive { get; set; }

        /// <summary>
        /// False holds a following NPC where it is (<c>10070503</c>): stock only lets it follow while
        /// the owner has bit 4 (<c>0x4</c>) of stat 224 set. N3Lite has no stats, so the owner sets
        /// this; <c>N3Lite.AORules</c> does it from the stat.
        /// </summary>
        public bool FollowEnabled { get; set; } = true;

        /// <summary>
        /// An NPC with nothing to follow still has a moving forward state, and should stop. Stock
        /// applies <c>FullStop</c> here, from the guide tick (<c>10070e39</c>), every tick until the
        /// state changes — so clear <see cref="ForwardActive"/> in response.
        /// </summary>
        public event Action FullStopRequested;

        /// <summary>
        /// Follows <paramref name="waypoints"/> in order (<c>SetFollowTarget</c>, <c>1007034b</c>, with
        /// the NPC as its own target — what a FollowTarget path does).
        ///
        /// <para>
        /// At most <see cref="FollowQueueCapacity"/> points are kept, and a <c>(0,0,0)</c> point ends
        /// the list, as stock's zero-terminated array does. The Y is kept: unlike
        /// <see cref="N3Lite.Path"/>, the queue is not flattened.
        /// </para>
        ///
        /// <para>
        /// <paramref name="snapToStart"/> reads the list as stock's message lays it out: the first
        /// point is where the NPC is now, so the body is snapped there (see <see cref="SnapTo"/>) and
        /// the queue is the points after it — or the start itself when nothing follows it. Pass false
        /// for a list that is waypoints only, as a server planning a path for an NPC in place would.
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
                if (!waypoints[0].IsZero)
                    SnapTo(waypoints[0]);

                // The start stays the first waypoint only when nothing follows it.
                if (waypoints.Count > 1 && !waypoints[1].IsZero)
                    first = 1;
            }

            for (int i = first; i < waypoints.Count && _followQueue.Count < FollowQueueCapacity; i++)
            {
                if (waypoints[i].IsZero)
                    break;
                _followQueue.Add(waypoints[i]);
            }
        }

        /// <summary>How far below the start point the snap looks for ground (<c>1015e7ac</c>).</summary>
        public const float SnapProbeDepth = 1.5f;

        /// <summary>
        /// Places the body at <paramref name="start"/>, dropped onto the ground when there is some
        /// within <see cref="SnapProbeDepth"/> below it, with no collision (<c>1007034b</c>, then
        /// <c>Vehicle_t::SetRelPosIgnoreCollision</c>).
        ///
        /// <para>
        /// Seam: stock casts that ray through the playfield's surface (<c>n3Playfield_t</c> <c>+0x60</c>,
        /// its slot 3), and <c>SetRelPosIgnoreCollision</c> was not read. This uses the vehicle's own
        /// surface and sets the position and previous position.
        /// </para>
        /// </summary>
        void SnapTo(Vec3 start)
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

        /// <summary>Stops following (<c>SetFollowTarget(null)</c>).</summary>
        public void ClearFollowTarget()
        {
            HasFollowTarget = false;
            _followQueue.Clear();
        }

        /// <summary>
        /// The follow point to steer at (<c>10070776</c>): drop every waypoint already within
        /// <see cref="FollowDistance"/> in XZ (<c>10070562</c>), then the first one left, or the body's
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
        /// Advances the guide (<c>NPCVehicle_t</c> slot 37, <c>10070e39</c>). It belongs to the AI
        /// tick, not to <c>Run</c>, so the caller drives it — once per frame with the frame's
        /// <c>dt</c>. Only a non-empty path advances, and the guide keeps the speed it was restarted
        /// with: stock never calls <c>PathGuide_t::UpdateMaxSpeed</c>. With no path, no follow target
        /// and a moving forward state it raises <see cref="FullStopRequested"/>.
        /// </summary>
        public void AdvanceGuide(float dt)
        {
            if (!Path.Empty)
                Guide.UpdateAddTime(dt);
            else if (!HasFollowTarget && ForwardActive)
                FullStopRequested?.Invoke();
        }

        /// <summary>
        /// Takes an NPC's motion as the spawn message carries it (<c>NPCVehicle_t</c> slot 43,
        /// <c>10070fea</c>): the velocity as is, then the path, and the guide restarted at
        /// <paramref name="guideTime"/> seconds along it at the current top speed. Set the movement
        /// state first, so the top speed is the right one. The client calls this; the server fills
        /// the message from <see cref="GetMotionState"/>.
        ///
        /// <para>
        /// A moving velocity is taken whole: an NPC already walking does not ramp up again. With no
        /// waypoints stock also applies <c>FullStop</c> to the movement status, which is the owner's.
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

            // The time is only in the message when there is a path.
            if (waypoints == null || waypoints.Count == 0)
                guideTime = 0f;

            Guide.RestartGuide(Path, MaxVel, guideTime);
        }

        /// <summary>
        /// The same motion read back out (<c>NPCVehicle_t</c> slot 42, <c>1007135a</c>): the velocity,
        /// the path's waypoints and the guide's time. Stock writes only the X and Z of each waypoint;
        /// the Y here is always 0 anyway.
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
        /// Stock steps an NPC's vehicle only while it has something to do (<c>NPCVehicle_t</c> slot 18,
        /// <c>10070e8e</c>): a path, a follow target, a moving forward state, or a fall. An idle NPC on
        /// the ground is not stepped at all. (Stock also runs one on a functional path, which N3Lite
        /// does not have.)
        /// </summary>
        protected override bool IsRunEnabled()
            => !Path.Empty || HasFollowTarget || ForwardActive || Airborne;

        /// <summary>
        /// <c>NPCVehicle_t::OnHalt</c> (<c>10071149</c>). A following NPC stops as any character does.
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
        /// The NPC's only steering channel (<c>10070ed6</c>): halt, walk the follow queue, or arrive at
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

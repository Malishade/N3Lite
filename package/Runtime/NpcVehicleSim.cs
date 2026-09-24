namespace N3Lite
{
    /// <summary>
    /// The vehicle an NPC moves with.
    ///
    /// <para>
    /// It is <b>not</b> a variant of the player vehicle. Two of the three steering channels do
    /// nothing: an NPC has <b>no strafe and no turn</b>. Its one channel follows a
    /// <see cref="N3Lite.Path"/> through a <see cref="PathGuide"/>, or arrives at a follow target.
    /// So the four input axes belong to the player alone.
    /// </para>
    ///
    /// <para>
    /// It still inherits everything in <see cref="CharVehicleSim"/> and <see cref="VehicleSim"/> —
    /// gravity, the ground clamp, statel collision, the orientation update, the speed curve.
    /// </para>
    /// </summary>
    public class NpcVehicleSim : CharVehicleSim
    {
        /// <summary>
        /// The waypoints. Empty means "stand still": the longitudinal channel halts rather than
        /// returning <see cref="SteeringResult.None"/>.
        /// </summary>
        public Path Path { get; } = new Path();

        /// <summary>The point that slides along <see cref="Path"/>, which the NPC steers at.</summary>
        public PathGuide Guide { get; } = new PathGuide();

        /// <summary>
        /// When set, the NPC arrives at <see cref="FollowTarget"/> and the path is ignored entirely.
        /// </summary>
        public bool HasFollowTarget { get; set; }

        /// <summary>The point a following NPC steers at.</summary>
        public Vec3 FollowTarget { get; set; }

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
        /// once per frame with the frame's <c>dt</c> is the straightforward choice.
        /// </summary>
        public void AdvanceGuide(float dt)
        {
            Guide.UpdateMaxSpeed(MaxVel);
            Guide.UpdateAddTime(dt);
        }

        /// <summary>
        /// The NPC's only steering channel: halt, arrive at the follow target, or arrive at the path
        /// guide.
        ///
        /// <para>
        /// The target's <b>Y is replaced by the body's own</b>, which keeps an NPC from steering up or
        /// down at a waypoint. Paths are flat anyway — <see cref="N3Lite.Path.AddWaypoint"/>
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
                return SteeringDirArrive(FollowTarget, out force);

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

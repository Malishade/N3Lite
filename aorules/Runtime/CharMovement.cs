using System;
using System.Collections.Generic;
using N3Lite;

namespace N3Lite.AORules
{
    /// <summary>
    /// A character's movement as the game runs it: the N3Lite core, with the movement state and the
    /// character's stats turned into its profile and jump by <see cref="CharMovementRules"/>.
    ///
    /// <para>
    /// Depends on N3Lite only, so a client and a server can run this same code. The protocol — which
    /// message means which state or flag — is each side's own.
    /// </para>
    ///
    /// <para>
    /// The state is the network's <c>MovementState</c> number; see <see cref="CharMovementRules"/>.
    /// </para>
    ///
    /// <para>
    /// Two ways to drive it. <see cref="Act"/> takes the wire's movement actions through
    /// <see cref="Status"/>, the game's own state machine, and runs each transition on the body: what
    /// a server checking a client's actions wants. <see cref="EnterState"/>,
    /// <see cref="LeaveState"/> and <see cref="Core"/>'s flags drive the body directly, for a side that
    /// keeps its own state.
    /// </para>
    /// </summary>
    public sealed class CharMovement
    {
        readonly N3CharVehicle _core;
        int _state = CharMovementRules.StateRun;
        MovementStats _stats;

        public CharMovement(
            bool isNpc = false, float mass = N3CharVehicle.DefaultMass, float radius = N3CharVehicle.DefaultRadius)
        {
            _core = new N3CharVehicle(isNpc, mass, radius);
            _core.JumpLanded += OnJumpLanded;
            ApplyVehicleStats();
        }

        /// <summary>The movement core — position, surface, flags, jump, path, the step.</summary>
        public N3CharVehicle Core => _core;

        public int State => _state;

        /// <summary>The movement status <see cref="Act"/> runs on.</summary>
        public CharMovementStatus Status { get; } = new CharMovementStatus();

        /// <summary>
        /// A player's body under Fear (spell function 53121) or Mind Control (53232): its own controls
        /// are dropped while set. The effects are the owner's, so the owner sets it; an NPC is never
        /// marked.
        /// </summary>
        public bool AiControlled { get; set; }

        /// <summary>
        /// The body is being moved along a server waypoint path: its own controls are dropped while set.
        /// The path is the owner's, so the owner sets it.
        /// </summary>
        public bool OnWaypointPath { get; set; }

        /// <summary>
        /// Bits to set in the character's Flags stat (stat 0): <see cref="CharMovementRules.FlagFlying"/>
        /// on taking to the air. The stats are the owner's, so the owner applies it.
        /// </summary>
        public event Action<int> FlagsSet;

        /// <summary>Bits to clear in the character's Flags stat (stat 0); see <see cref="FlagsSet"/>.</summary>
        public event Action<int> FlagsCleared;

        /// <summary>Walk or Run, whichever was last active; <see cref="LeaveState"/> returns to it.</summary>
        public int LastSpeedMode { get; set; } = CharMovementRules.StateRun;

        public MovementStats Stats => _stats;

        /// <summary>Advance the body by <paramref name="dt"/> seconds.</summary>
        public void Tick(float dt) => _core.Tick(dt);

        /// <summary>
        /// Whether this body moves as a player (four input axes) or as an NPC (follows a path) — the
        /// spawn message's <c>IsNpc</c>.
        /// </summary>
        public void SelectVehicleKind(bool isNpc)
        {
            _core.SelectVehicleKind(isNpc);
            ApplyVehicleStats();
        }

        /// <summary>
        /// The owner's stats: the run speed into the speed curves, the jump height from Strength,
        /// Agility and GmLevel, the body height from the scale, and the vehicle's orientation mode from
        /// stat 197.
        /// </summary>
        public void SetStats(MovementStats stats)
        {
            _stats = stats;
            _core.JumpHeight = CharMovementRules.JumpHeight(stats.Strength, stats.Agility, stats.GmLevel);
            _core.BodyHeight = CharMovementRules.BodyHeight(stats.BodyScale);
            ApplyVehicleStats();
        }

        /// <summary>
        /// The character's stats onto its vehicle: the orientation mode
        /// (<see cref="CharMovementRules.OrientationMode"/>), then the rules for the current state — and
        /// in Fly, falling off and the flying orientation again, as a status read takes it.
        /// </summary>
        void ApplyVehicleStats()
        {
            _core.Vehicle.SetOrientationMode(CharMovementRules.OrientationMode(_stats.OrientationMode));
            ApplyStatusState(_state);
        }

        /// <summary>
        /// A transition into <paramref name="state"/>. Into Fly it runs <see cref="StartFlying"/>
        /// (again, if already flying); from Fly into Walk, Run or Swim it runs
        /// <see cref="StopFlying"/>. From Fly into any other state the body stays as it flew — falling
        /// off and pitched along its velocity — until one of those, or <see cref="LeaveFly"/>.
        /// </summary>
        public void EnterState(int state)
        {
            int previous = _state;
            if (_state == CharMovementRules.StateWalk || _state == CharMovementRules.StateRun)
                LastSpeedMode = _state;

            SetMode(state);
            ApplyRules();

            if (state == CharMovementRules.StateFly)
                StartFlying();
            else if (previous == CharMovementRules.StateFly
                     && (state == CharMovementRules.StateWalk
                         || state == CharMovementRules.StateRun
                         || state == CharMovementRules.StateSwim))
                StopFlying();
        }

        public void LeaveState()
            => EnterState(LeaveTarget);

        /// <summary>
        /// Leaving fly, back to the last gait: <see cref="StopFlying"/> whatever the state was, so it
        /// also lands a body that went from Fly into another state.
        /// </summary>
        public void LeaveFly()
        {
            SetMode(LeaveTarget);
            ApplyRules();
            StopFlying();
        }

        int LeaveTarget
            => LastSpeedMode == CharMovementRules.StateWalk || LastSpeedMode == CharMovementRules.StateRun
                ? LastSpeedMode
                : CharMovementRules.StateRun;

        /// <summary>
        /// <paramref name="state"/> taken as it is, with no transition — a status read. In Fly the body
        /// stops falling and is pitched along its velocity, but is not lifted; out of Fly nothing about
        /// falling or orientation changes.
        /// </summary>
        public void ApplyStatusState(int state)
        {
            SetMode(state);
            ApplyRules();

            if (state == CharMovementRules.StateFly)
            {
                FlagsSet?.Invoke(CharMovementRules.FlagFlying);
                _core.Vehicle.DisableFalling();
                _core.Vehicle.SetOrientationMode(CharMovementRules.FlyOrientationMode);
            }
        }

        void SetMode(int state)
        {
            _state = state;
            Status.Current.Mode = state;
        }

        // ---- actions -----------------------------------------------------------

        /// <summary>
        /// A movement action (the wire's <c>MovementAction</c> number) from the character's own
        /// controls. Returns false, changing nothing, when the current mode refuses it
        /// (<see cref="CharMovementStatus.Accepts"/>) or the controls drop it
        /// (<see cref="CharMovementRules.DropsControl"/>: no jump while flying); a server drops such an
        /// action rather than
        /// passing it on. Otherwise the status takes it (<see cref="CharMovementStatus.Apply"/>) and
        /// the transition it names runs on the body. One it does not name leaves the body alone,
        /// though the status may still have changed.
        ///
        /// <para>
        /// The jump's end (0x10) is not the controls': it runs by itself when a jump lands. Act also
        /// takes the turn rate over from the flags (<see cref="N3CharVehicle.TurnRateFromFlags"/>): it
        /// is set by the turn and drive transitions, from <see cref="MovementStats.TurnSpeed"/>.
        /// </para>
        /// </summary>
        public bool Act(int action)
        {
            if (OnWaypointPath || (AiControlled && !_core.IsNpcVehicle)
                || !Status.Accepts(action)
                || CharMovementRules.DropsControl(Status.Current.Mode, action, _stats.MechData))
                return false;

            Apply(action);
            return true;
        }

        /// <summary>
        /// <see cref="Act"/> without the mode's refusal: an action the game itself raises, such as the
        /// jump's end.
        /// </summary>
        public void Apply(int action)
        {
            _core.TurnRateFromFlags = false;
            CharMovementStatus.Transition transition =
                Status.Apply(action, CharMovementRules.CanFollow(_stats.Features), out CharMovementStatus.Fields before);
            Run(transition, before);
        }

        /// <summary>
        /// A whole status as a status message carries it, taken without a transition: the mode as
        /// <see cref="ApplyStatusState"/> takes it, the last gait, and the axes onto the body's flags.
        /// </summary>
        public void ApplyStatus(CharMovementStatus.Fields fields, int lastSpeedMode)
        {
            Status.Current = fields;
            Status.LastSpeedMode = lastSpeedMode;
            LastSpeedMode = lastSpeedMode;
            _core.TurnRateFromFlags = false;
            ApplyStatusState(fields.Mode);
            _core.SetFlags(FlagsOf(Status.Current));
            SetKeyboardTurnRate();
        }

        /// <summary>
        /// A landing: while the status is in a jump the jump's end runs at once, inside the landing
        /// as the game runs it. In Fly that is what makes a jump do nothing: the jump turns falling
        /// back on, which lands the body where it is, and the jump's end turns it off again.
        /// </summary>
        void OnJumpLanded()
        {
            if (Status.Current.JumpState == CharMovementStatus.Airborne)
                Apply(0x10);
        }

        /// <summary>
        /// The turn rate of a keyboard turn in the current status (<see cref="CharMovementRules.TurnRate"/>),
        /// or 0 when not turning.
        /// </summary>
        void SetKeyboardTurnRate()
        {
            CharMovementStatus.Fields s = Status.Current;
            float rate = s.TurnState == CharMovementStatus.Idle
                ? 0f
                : CharMovementRules.TurnRate(
                    _stats.TurnSpeed, s.TurnDir == CharMovementStatus.DirLeft, s.FwdState == CharMovementStatus.Idle);
            _core.Vehicle.SetTurnRate(rate);
        }

        void Run(CharMovementStatus.Transition transition, CharMovementStatus.Fields before)
        {
            int mode = Status.Current.Mode;

            switch (transition)
            {
                case CharMovementStatus.Transition.None:
                    // Some actions write the mode even when they are refused; nothing runs for them.
                    _state = mode;
                    return;

                case CharMovementStatus.Transition.ElevateUpStart:
                    ElevateUpStart();
                    return;

                case CharMovementStatus.Transition.ElevateUpStop:
                    ElevateUpStop();
                    return;

                case CharMovementStatus.Transition.SwitchToFly:
                    _state = mode;
                    ApplyRules();
                    StartFlying();
                    return;

                case CharMovementStatus.Transition.LeaveFly:
                    _state = mode;
                    ApplyRules();
                    StopFlying();
                    return;

                case CharMovementStatus.Transition.SwitchToWalk:
                case CharMovementStatus.Transition.SwitchToRun:
                case CharMovementStatus.Transition.SwitchToSwim:
                    _state = mode;
                    if (mode == CharMovementRules.StateWalk || mode == CharMovementRules.StateRun)
                    {
                        Status.LastSpeedMode = mode;
                        LastSpeedMode = mode;
                    }
                    ApplyRules();
                    if (before.Mode == CharMovementRules.StateFly)
                        StopFlying();
                    return;

                case CharMovementStatus.Transition.SwitchToFrozen:
                    _state = mode;
                    ApplyRules();
                    _core.SetFlags(FlagsOf(Status.Current));
                    // A player's body stops turning when frozen.
                    if (!_core.IsNpcVehicle)
                        _core.Vehicle.SetTurnRate(0f);
                    return;

                case CharMovementStatus.Transition.SwitchToCrawl:
                case CharMovementStatus.Transition.SwitchToSneak:
                case CharMovementStatus.Transition.SwitchToSitGround:
                case CharMovementStatus.Transition.SwitchToSleep:
                case CharMovementStatus.Transition.SwitchToLounge:
                case CharMovementStatus.Transition.LeaveSwim:
                case CharMovementStatus.Transition.LeaveSneak:
                case CharMovementStatus.Transition.LeaveSit:
                case CharMovementStatus.Transition.LeaveFrozen:
                case CharMovementStatus.Transition.LeaveCrawl:
                case CharMovementStatus.Transition.LeaveSleep:
                case CharMovementStatus.Transition.LeaveLounge:
                    _state = mode;
                    ApplyRules();
                    break;

                case CharMovementStatus.Transition.TurnLeftStart:
                case CharMovementStatus.Transition.TurnRightStart:
                    _core.SetFlags(FlagsOf(Status.Current));
                    SetKeyboardTurnRate();
                    return;

                // A mouse turn turns the body by the mouse: no turn rate.
                case CharMovementStatus.Transition.MouseTurnLeftStart:
                case CharMovementStatus.Transition.MouseTurnRightStart:
                case CharMovementStatus.Transition.TurnStop:
                case CharMovementStatus.Transition.FullStop:
                    _core.SetFlags(FlagsOf(Status.Current));
                    _core.Vehicle.SetTurnRate(0f);
                    return;

                // Starting or stopping the drive mid-turn re-rates a keyboard turn on a player's body;
                // a turn at rate 0 (a mouse turn) is left alone.
                case CharMovementStatus.Transition.ForwardStart:
                case CharMovementStatus.Transition.ForwardStop:
                case CharMovementStatus.Transition.ReverseStart:
                case CharMovementStatus.Transition.ReverseStop:
                    _core.SetFlags(FlagsOf(Status.Current));
                    if (!_core.IsNpcVehicle && _core.Vehicle.TurnRate != 0f
                        && Status.Current.TurnState != CharMovementStatus.Idle)
                        SetKeyboardTurnRate();
                    return;

                case CharMovementStatus.Transition.JumpStart:
                    _core.SetFlags(FlagsOf(Status.Current));
                    _core.TryStartJump();
                    return;

                case CharMovementStatus.Transition.JumpStop:
                    ApplyRules();
                    break;
            }

            _core.SetFlags(FlagsOf(Status.Current));
        }

        /// <summary>
        /// The body's input flags for a status: drive, strafe and turn from the axes, and the jump
        /// while airborne. The turn flags do not set the turn rate here; the transitions do.
        /// </summary>
        MovementFlags FlagsOf(CharMovementStatus.Fields s)
        {
            MovementFlags flags = MovementFlags.None;

            if (s.FwdState == CharMovementStatus.Active)
            {
                if (s.FwdDir == CharMovementStatus.DirForward)
                    flags |= MovementFlags.Forward;
                else if (s.FwdDir == CharMovementStatus.DirReverse)
                    flags |= MovementFlags.Backward;
            }

            if (s.StrafeState == CharMovementStatus.Active)
            {
                if (s.StrafeDir == CharMovementStatus.DirLeft)
                    flags |= MovementFlags.StrafeLeft;
                else if (s.StrafeDir == CharMovementStatus.DirRight)
                    flags |= MovementFlags.StrafeRight;
            }

            if (s.TurnState == CharMovementStatus.Turning)
            {
                if (s.TurnDir == CharMovementStatus.DirLeft)
                    flags |= MovementFlags.TurnLeft;
                else if (s.TurnDir == CharMovementStatus.DirRight)
                    flags |= MovementFlags.TurnRight;
            }

            if (s.JumpState == CharMovementStatus.Airborne)
                flags |= MovementFlags.Jump;

            return flags;
        }

        // ---- flying ----------------------------------------------------------

        /// <summary>
        /// Starts rising at <see cref="CharMovementRules.ElevateUpSpeed"/>. The caller's state machine
        /// decides when it applies: only in Fly.
        /// </summary>
        public void ElevateUpStart()
        {
            ApplyRules();
            _core.Vehicle.SetVertical(CharMovementRules.ElevateUpSpeed);
        }

        /// <summary>
        /// Stops rising: the body sinks at <see cref="CharMovementRules.ElevateStopSpeed"/>. The
        /// caller's state machine decides when it applies: only after <see cref="ElevateUpStart"/>.
        /// </summary>
        public void ElevateUpStop()
            => _core.Vehicle.SetVertical(CharMovementRules.ElevateStopSpeed);

        /// <summary>
        /// Into Fly: falling off, the body lifted <see cref="CharMovementRules.FlyStartLift"/> (only once
        /// it is in a world — a surface is set) and pitched along its velocity.
        /// </summary>
        void StartFlying()
        {
            FlagsSet?.Invoke(CharMovementRules.FlagFlying);

            CharVehicleSim vehicle = _core.Vehicle;
            vehicle.DisableFalling();

            if (vehicle.Surface != null)
                vehicle.SetRelPos(vehicle.Position + new Vec3(0f, CharMovementRules.FlyStartLift, 0f));

            vehicle.SetOrientationMode(CharMovementRules.FlyOrientationMode);
        }

        /// <summary>
        /// Out of Fly: falling back on, the body lifted <see cref="CharMovementRules.FlyLeaveLift"/>, the
        /// vertical axis cleared on a player's body, and the orientation mode set from
        /// <see cref="MovementStats.OrientationMode"/>.
        /// </summary>
        void StopFlying()
        {
            FlagsCleared?.Invoke(CharMovementRules.FlagFlying);

            CharVehicleSim vehicle = _core.Vehicle;
            vehicle.EnableFalling();
            vehicle.SetRelPos(vehicle.Position + new Vec3(0f, CharMovementRules.FlyLeaveLift, 0f));

            if (!_core.IsNpcVehicle)
                vehicle.SetVertical(0f);

            vehicle.SetOrientationMode(_stats.OrientationMode);
        }

        /// <summary>
        /// A follow path as the game's messages lay it out, onto an NPC body: the first point is where
        /// the NPC is now, and the rest are its waypoints. The list is zero-terminated — a
        /// <c>(0,0,0)</c> point ends it — and holds at most <see cref="FollowPathCapacity"/> waypoints.
        /// <paramref name="snapToStart"/> false reads the list as waypoints only. Returns false on a
        /// player's body. See <see cref="N3CharVehicle.SetPath"/>.
        /// </summary>
        public bool SetFollowPath(IReadOnlyList<Vec3> points, bool snapToStart)
        {
            var waypoints = new List<Vec3>();
            Vec3? start = null;

            if (points != null && points.Count > 0)
            {
                int first = 0;
                if (snapToStart)
                {
                    if (!points[0].IsZero)
                        start = points[0];

                    // The start stays the first waypoint only when nothing follows it.
                    if (points.Count > 1 && !points[1].IsZero)
                        first = 1;
                }

                for (int i = first; i < points.Count && waypoints.Count < FollowPathCapacity; i++)
                {
                    if (points[i].IsZero)
                        break;
                    waypoints.Add(points[i]);
                }
            }

            if (!_core.SetPath(waypoints))
                return false;

            if (start.HasValue)
                _core.Npc.SnapTo(start.Value);
            return true;
        }

        /// <summary>The most waypoints a follow path message carries.</summary>
        public const int FollowPathCapacity = 30;

        /// <summary>
        /// The state, the run speed and the vehicle kind, into the core's profile; and on an NPC, stat
        /// 224's follow bit.
        /// </summary>
        void ApplyRules()
        {
            _core.SetProfile(CharMovementRules.Profile(_state, _stats.RunSpeed, _core.IsNpcVehicle));

            if (_core.Npc != null)
                _core.Npc.FollowEnabled = CharMovementRules.CanFollow(_stats.Features);
        }
    }
}

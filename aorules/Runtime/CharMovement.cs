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
            ApplyRules();
        }

        /// <summary>The movement core — position, surface, flags, jump, path, the step.</summary>
        public N3CharVehicle Core => _core;

        public int State => _state;

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
            ApplyRules();
        }

        /// <summary>
        /// The owner's stats: the run speed into the speed curves, the jump height from Strength,
        /// Agility and GmLevel, and the body height from the scale.
        /// </summary>
        public void SetStats(MovementStats stats)
        {
            _stats = stats;
            _core.JumpHeight = CharMovementRules.JumpHeight(stats.Strength, stats.Agility, stats.GmLevel);
            _core.BodyHeight = CharMovementRules.BodyHeight(stats.BodyScale);
            ApplyRules();
        }

        public void EnterState(int state)
        {
            if (_state == CharMovementRules.StateWalk || _state == CharMovementRules.StateRun)
                LastSpeedMode = _state;

            _state = state;
            ApplyRules();
        }

        public void LeaveState()
            => EnterState(LastSpeedMode == CharMovementRules.StateWalk || LastSpeedMode == CharMovementRules.StateRun
                ? LastSpeedMode
                : CharMovementRules.StateRun);

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

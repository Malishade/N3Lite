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
        /// Which of <c>CharVehicle_t</c>'s two subclasses this body is — the spawn message's
        /// <c>IsNpc</c>. See N3Lite/docs/Movement.md §3.2.
        /// </summary>
        public void SelectVehicleKind(bool isNpc)
        {
            _core.SelectVehicleKind(isNpc);
            ApplyRules();
        }

        /// <summary>
        /// The owner's stats: the run speed into the speed curves, the jump height from Strength,
        /// Agility and GmLevel, and the body height from the scale.
        ///
        /// <para>
        /// The health penalty an older file applied is <b>not</b> here: it was invented, and stock's
        /// stat lookup has not been reversed far enough to say whether one exists.
        /// </para>
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

using System;
using System.Collections.Generic;
using N3Lite.Surfaces;

namespace N3Lite
{
    /// <summary>
    /// A character's movement: the vehicle plus what drives it — input flags to the four axes, a
    /// <see cref="MovementProfile"/> for its speeds and permissions, the jump and NPC paths. Call
    /// <see cref="Tick"/> once per frame.
    /// </summary>
    public class N3CharVehicle
    {
        public const float DefaultMass = 50f;
        public const float DefaultRadius = 0.5f;

        static readonly MovementFlags TranslationFlags =
            MovementFlags.Forward | MovementFlags.Backward |
            MovementFlags.StrafeLeft | MovementFlags.StrafeRight;

        readonly float _mass;
        readonly float _radius;

        CharVehicleSim _sim;
        bool _isNpc;
        MovementFlags _flags;
        MovementProfile _profile = MovementProfile.Default;
        bool _reversing;
        float _bodyHeight = 2f;

        /// <summary>A jump was taken.</summary>
        public event Action JumpStarted;

        /// <summary>A jump in progress landed.</summary>
        public event Action JumpLanded;

        /// <summary>The path was cleared — any path the caller keeps alongside should be cleared too.</summary>
        public event Action PathCleared;

        /// <summary>Keyboard turn rate while moving, radians per second.</summary>
        public float TurnRateMoving = 1.5f;

        /// <summary>Keyboard turn rate while standing, radians per second.</summary>
        public float TurnRateStopped = 3.5f;

        public N3CharVehicle(bool isNpc = false, float mass = DefaultMass, float radius = DefaultRadius)
        {
            _mass = mass;
            _radius = radius;
            Build(isNpc);
        }

        public CharVehicleSim Vehicle => _sim;

        /// <summary>The NPC vehicle, or null for a player's body.</summary>
        public NpcVehicleSim Npc => _sim as NpcVehicleSim;

        /// <summary>
        /// An NPC body has no strafe and no turn channel and follows a path; a player's body has the
        /// four input axes.
        /// </summary>
        public bool IsNpcVehicle => _isNpc;

        public ISurface Surface
        {
            get => _sim.Surface;
            set => _sim.Surface = value;
        }

        public Vec3 Position
        {
            get => _sim.Position;
            set => _sim.Position = value;
        }

        public MovementFlags Flags => _flags;

        public MovementProfile Profile => _profile;

        /// <summary>The height of the next jump, m.</summary>
        public float JumpHeight = 1f;

        /// <summary>The body's height, which a jump keeps clear of a ceiling.</summary>
        public float BodyHeight
        {
            get => _bodyHeight;
            set
            {
                _bodyHeight = value;
                _sim.BodyHeight = value;
            }
        }

        public float CurrentSpeed => _sim.Speed;

        public float MaxVel => _sim.MaxVel;

        public float MaxForce => _sim.MaxForce;

        public bool IsTranslating => (_flags & TranslationFlags) != 0;

        public bool IsMoving => CurrentSpeed > 0.001f || IsTranslating;

        // ---- building ----------------------------------------------------------

        /// <summary>
        /// Switches between the player and the NPC vehicle. The position and surface carry over; the
        /// rotation starts fresh, so warp the body afterwards.
        /// </summary>
        public void SelectVehicleKind(bool isNpc)
        {
            if (isNpc == _isNpc)
                return;

            Build(isNpc);
        }

        void Build(bool isNpc)
        {
            CharVehicleSim previous = _sim;
            if (previous != null)
                previous.JumpLanded -= OnVehicleJumpLanded;

            _isNpc = isNpc;
            _sim = isNpc ? new NpcVehicleSim() : new CharVehicleSim();
            _sim.Mass = _mass;
            _sim.MaxForce = 10f;
            _sim.MaxVel = 1f;
            _sim.NearProbeOffset = _radius;
            _sim.SlowingDistance = 1.5f;
            _sim.BodyHeight = _bodyHeight;
            _sim.JumpLanded += OnVehicleJumpLanded;

            _sim.EnableFalling();
            _sim.DisableSurfaceHug();

            // Aligns the body to the ground, which tilts its forward along a slope — the tilt is what
            // lets the swept move see an uphill move and refuse it.
            _sim.UseSurfaceNormal();

            _reversing = false;
            ApplyProfileToSim(wasFlying: false);

            if (previous != null)
            {
                _sim.Position = previous.Position;
                _sim.Surface = previous.Surface;
            }
        }

        // ---- the frame -----------------------------------------------------------

        /// <summary>Advances an NPC's path guide, then steps the vehicle.</summary>
        public void Tick(float dt)
        {
            Npc?.AdvanceGuide(dt);
            _sim.Run(dt);
        }

        // ---- input ---------------------------------------------------------------

        /// <summary>
        /// A player's input. Returns false, doing nothing, on an NPC body, which has no input axes.
        /// Clears any path; when the profile refuses input it only clears the flags; a new jump flag
        /// starts a jump.
        /// </summary>
        public bool SetInputs(MovementFlags flags)
        {
            if (_isNpc)
                return false;

            ClearPath();

            if (!_profile.AcceptsInput)
            {
                SetFlags(MovementFlags.None);
                return true;
            }

            bool jumpRising = (flags & MovementFlags.Jump) != 0 && (_flags & MovementFlags.Jump) == 0;

            SetFlags(flags);

            if (jumpRising)
                TryStartJump();

            return true;
        }

        /// <summary>Stores the flags and applies them to the axes (a no-op on an NPC body).</summary>
        public void SetFlags(MovementFlags flags)
        {
            _flags = flags;
            ApplyFlagsToAxes();
        }

        void ApplyFlagsToAxes()
        {
            // An NPC body has no input axes: the flags are stored for whoever reads them, but writing
            // axes would fight the path guide.
            if (_isNpc)
                return;

            float drive = 0f;
            if ((_flags & MovementFlags.Forward) != 0)
                drive += 1f;
            if ((_flags & MovementFlags.Backward) != 0)
                drive -= 1f;

            // Backing up switches to the reverse speeds.
            bool reversing = drive < 0f;
            if (_reversing != reversing)
            {
                _reversing = reversing;
                ApplySpeeds();
            }

            // Every drive change is paired with SetDirection, and a release also halts:
            //   forward   SetDirection(1);  SetForwardDrive(1)
            //   backward  SetForwardDrive(-1);  SetDirection(-1)
            //   release   SetForwardDrive(0);  Halt();  SetDirection(1)
            // Without the halt the body coasts at its last speed; without the direction a backing
            // body turns to face its velocity and oscillates on the spot.
            if (drive != _sim.ForwardDrive)
            {
                if (drive > 0f)
                {
                    _sim.SetDirection(1);
                    _sim.SetForwardDrive(drive);
                }
                else if (drive < 0f)
                {
                    _sim.SetForwardDrive(drive);
                    _sim.SetDirection(-1);
                }
                else
                {
                    _sim.SetForwardDrive(0f);
                    _sim.Halt();
                    _sim.SetDirection(1);
                }
            }

            // The strafe setter keeps only the sign.
            float strafe = 0f;
            if ((_flags & MovementFlags.StrafeRight) != 0)
                strafe += 1f;
            if ((_flags & MovementFlags.StrafeLeft) != 0)
                strafe -= 1f;
            _sim.SetStrafe(strafe);

            float turn = 0f;
            if ((_flags & MovementFlags.TurnRight) != 0)
                turn += 1f;
            if ((_flags & MovementFlags.TurnLeft) != 0)
                turn -= 1f;
            _sim.SetTurnRate(turn * (IsMoving ? TurnRateMoving : TurnRateStopped));
        }

        // ---- the profile ---------------------------------------------------------

        /// <summary>
        /// Sets the speeds and permissions. Moving into a profile that does not accept input clears
        /// the flags and the path.
        /// </summary>
        public void SetProfile(MovementProfile profile)
        {
            if (!profile.AcceptsInput && _profile.AcceptsInput)
            {
                SetFlags(MovementFlags.None);
                ClearPath();
            }

            bool wasFlying = _profile.Flying;
            _profile = profile;
            ApplyProfileToSim(wasFlying);
            ApplyFlagsToAxes();
        }

        void ApplyProfileToSim(bool wasFlying)
        {
            _sim.DriveLocked = !_profile.CanDrive;
            ApplySpeeds();

            if (_profile.Flying)
                _sim.DisableFalling();
            else if (wasFlying)
                _sim.EnableFalling();
        }

        void ApplySpeeds()
        {
            _sim.UpdateMotionConstraints(_reversing ? _profile.ReverseSpeed : _profile.ForwardSpeed);
            _sim.StrafeSpeed = _reversing ? _profile.ReverseStrafeSpeed : _profile.StrafeSpeed;
        }

        // ---- the jump --------------------------------------------------------------

        /// <summary>
        /// Jumps <see cref="JumpHeight"/> unless the profile refuses input or a jump is in progress;
        /// raises <see cref="JumpStarted"/> when it does.
        /// </summary>
        public bool TryStartJump()
        {
            if (!_profile.AcceptsInput)
                return false;

            if (!_sim.Jump(JumpHeight))
                return false;

            JumpStarted?.Invoke();
            return true;
        }

        void OnVehicleJumpLanded()
        {
            _flags &= ~MovementFlags.Jump;
            JumpLanded?.Invoke();
        }

        // ---- placement -------------------------------------------------------------

        public void Halt() => _sim.Halt();

        /// <summary>
        /// Turns the body to <paramref name="rotation"/>, re-aiming the velocity along the new facing —
        /// the way to change a moving body's heading.
        /// </summary>
        public void SetRelRot(Quat rotation) => _sim.SetRelRot(rotation);

        /// <summary>
        /// Places the body without steering. <paramref name="forward"/> is the new facing;
        /// <paramref name="resetVelocity"/> also halts it and clears the input flags.
        /// </summary>
        public void Warp(Vec3 position, Vec3 forward, bool resetVelocity = true)
        {
            _sim.Position = position;
            _sim.SetRelRot(Quat.LookRotation(forward, _sim.SurfaceNormal));

            if (resetVelocity)
            {
                _sim.Halt();
                SetFlags(MovementFlags.None);
            }
        }

        // ---- paths -----------------------------------------------------------------

        /// <summary>
        /// Clears the path and the input flags, then — on an NPC body — follows
        /// <paramref name="waypoints"/>. Returns false on a player's body, which has no path of its
        /// own; the caller steers those.
        /// </summary>
        public bool SetPath(IReadOnlyList<Vec3> waypoints)
        {
            ClearPath();
            SetFlags(MovementFlags.None);

            NpcVehicleSim npc = Npc;
            if (npc == null)
                return false;

            if (waypoints == null || waypoints.Count == 0)
                return true;

            npc.Path.Clear();
            for (int i = 0; i < waypoints.Count; i++)
                npc.Path.AddWaypoint(waypoints[i]);
            npc.RestartPath();
            return true;
        }

        public void ClearPath()
        {
            Npc?.Path.Clear();
            PathCleared?.Invoke();
        }
    }
}

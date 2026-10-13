using System;
using N3Lite;

namespace N3Lite.AORules
{
    /// <summary>
    /// The game's movement rules, turned into the plain numbers N3Lite takes: the per-state speed
    /// curves driven by the run-speed stat, the strafe rule, which states lock the drive, and the jump
    /// height from stats.
    ///
    /// <para>
    /// The state is the network's <c>MovementState</c> number. Only some numbers have names here: 2,
    /// 3 and 4 are what switching to walk, run and swim set, 7 is Fly, and 1, 8 and 9 refuse forward
    /// drive. The names behind the others are not known, so it stays an <c>int</c>.
    /// </para>
    /// </summary>
    public static class CharMovementRules
    {
        public const int StateRooted = 1;
        public const int StateWalk = 2;
        public const int StateRun = 3;
        public const int StateSwim = 4;
        public const int StateFly = 7;
        public const int StateSit = 8;

        /// <summary>
        /// The profile for <paramref name="state"/> at <paramref name="runSpeedStat"/>.
        ///
        /// <para>
        /// A player's vehicle refuses longitudinal steering in states 1, 8 and 9; an NPC's only in 1,
        /// where it halts instead. Sit (8) also drops input and refuses
        /// jumps, which is the character's rule rather than the vehicle's.
        /// </para>
        /// </summary>
        public static MovementProfile Profile(int state, float runSpeedStat, bool isNpc) => new MovementProfile
        {
            ForwardSpeed = MaxSpeed(state, 1, runSpeedStat),
            ReverseSpeed = MaxSpeed(state, 2, runSpeedStat),
            StrafeSpeed = StrafeSpeed(state, 1, runSpeedStat),
            ReverseStrafeSpeed = StrafeSpeed(state, 2, runSpeedStat),
            CanDrive = isNpc
                ? state != StateRooted
                : state != 9 && state != StateSit && state != StateRooted,
            AcceptsInput = state != StateSit,
            Flying = state == StateFly,
        };

        struct Curve
        {
            public float Divisor;
            public float Base;
            public float Max;
            public float Min;
            public bool Constant;
        }

        /// <summary>The per-state speed curve. <paramref name="direction"/> 2 is the reverse curve.</summary>
        static Curve CurveFor(int state, int direction)
        {
            switch (state)
            {
                case 3:
                    return direction == 2
                        ? new Curve { Divisor = 275f / 0.7f, Base = 3f, Max = 9.099999f, Min = 1.05f }
                        : new Curve { Divisor = 275f, Base = 5f, Max = 13f, Min = 1.5f };
                case 4:
                    return new Curve { Divisor = 275f / 0.625f, Base = 3f, Max = 8f, Min = 1.5f };
                case 7:
                    return new Curve { Divisor = 275f, Base = 7f, Max = 15f, Min = 1.5f };
                case 5:
                    return new Curve { Base = 1f, Constant = true };
                default:
                    // state 2 and everything unlisted
                    return new Curve { Base = 1.5f, Constant = true };
            }
        }

        /// <summary>
        /// The top speed: <c>stat / divisor + base</c> clamped to the curve's range, or the constant.
        /// The vehicle floors anything under 0.01 to 0.1 (<see cref="CharVehicleSim.UpdateMotionConstraints"/>).
        /// </summary>
        public static float MaxSpeed(int state, int direction, float runSpeedStat)
        {
            Curve curve = CurveFor(state, direction);
            if (curve.Constant)
                return curve.Base;

            float v = runSpeedStat / curve.Divisor + curve.Base;
            if (v > curve.Max)
                v = curve.Max;
            if (v < curve.Min)
                v = curve.Min;
            return v;
        }

        /// <summary>
        /// The strafe speed: the curve <b>scaled by 0.5</b>, with a floor of 0.75 and no separate
        /// maximum — <c>clamp(0.5*stat/divisor + 0.5*base, 0.75, 0.5*max)</c>. For the run state that
        /// is <c>clamp(stat*0.5/275 + 2.5, 0.75, 6.5)</c>.
        /// </summary>
        public static float StrafeSpeed(int state, int direction, float runSpeedStat)
        {
            const float Scale = 0.5f;
            const float Floor = 0.75f;

            if (state == StateWalk)
                return Math.Max(Floor, 1.5f);

            Curve curve = CurveFor(state, direction);
            if (curve.Constant)
                return Math.Max(Floor, curve.Base);

            float v = Scale * runSpeedStat / curve.Divisor + Scale * curve.Base;
            float max = Scale * curve.Max;
            if (v > max)
                v = max;
            if (v < Floor)
                v = Floor;
            return v;
        }

        /// <summary>
        /// A character's jump height from Strength (stat 16), Agility (17) and GmLevel (215):
        /// <c>(str + agi) / 200 + 1</c>, at least 0.5. Past 800 in total a non-GM counts as exactly
        /// 800 (<c>str = 800, agi = 0</c>).
        /// </summary>
        public static float JumpHeight(int strength, int agility, int gmLevel)
        {
            float str = strength;
            float agi = agility;
            if (800f < agi + str && gmLevel == 0)
            {
                str = 800f;
                agi = 0f;
            }

            float height = (float)((agi + str) / 200.0 + 1.0);
            if (height < 0.5f)
                height = 0.5f;
            return height;
        }

        /// <summary>Stat 224 bit 4 (<c>0x4</c>): an NPC may follow.</summary>
        public const int FeatureCanFollow = 0x4;

        /// <summary>Whether an NPC with these stat-224 features may follow its path or target.</summary>
        public static bool CanFollow(int features) => (features & FeatureCanFollow) != 0;

        /// <summary>
        /// A keyboard turn's rate, radians per second, negative to the left. With a turn speed (stat
        /// 267) of 0 it is 3.5 standing and 1.5 driving; otherwise it is <c>stat / 11000</c> standing and
        /// half that driving. Standing means the drive axis is idle, not that the body is at rest.
        /// </summary>
        public static float TurnRate(int turnSpeedStat, bool left, bool standing)
        {
            if (turnSpeedStat == 0)
            {
                float rate = standing ? 3.5f : 1.5f;
                return left ? -rate : rate;
            }

            float r = (float)(turnSpeedStat / 11000.0);
            if (left)
                r = -r;
            return standing ? r : (float)(r * 0.5);
        }

        /// <summary>
        /// The character's own controls are dropped before the status sees them, beyond what the mode
        /// refuses (<see cref="CharMovementStatus.Accepts"/>): no jump while flying or while MechData
        /// (stat 662) is set, and no switch to sneak while sitting.
        /// </summary>
        public static bool DropsControl(int mode, int action, int mechData)
        {
            const int JumpStart = 0xf;
            const int SwitchToSneak = 0x1c;

            if (action == JumpStart && (mode == StateFly || mechData != 0))
                return true;
            return mode == StateSit && action == SwitchToSneak;
        }

        /// <summary>The ceiling clamp keeps twice the body scale clear above the position.</summary>
        public static float BodyHeight(float bodyScale) => bodyScale + bodyScale;

        // ---- flying ----------------------------------------------------------

        /// <summary>The vertical axis while elevating up, m/s along world up.</summary>
        public const float ElevateUpSpeed = 3f;

        /// <summary>
        /// The vertical axis once elevating up stops: a flying body sinks slowly rather than hovering.
        /// </summary>
        public const float ElevateStopSpeed = -0.8f;

        /// <summary>How far switching to fly lifts the body, m.</summary>
        public const float FlyStartLift = 0.5f;

        /// <summary>How far leaving fly lifts the body before it falls again, m.</summary>
        public const float FlyLeaveLift = 0.1f;

        /// <summary>
        /// A character's vehicle orientation mode from stat 197: the stat as it is, except that 2 is
        /// taken as 1. 0 keeps the body level, 1 tilts it to the ground. Leaving fly takes the stat
        /// as it is, 2 included.
        /// </summary>
        public static int OrientationMode(int stat) => stat == 2 ? 1 : stat;

        /// <summary>The bit flying sets in the character's Flags stat (stat 0).</summary>
        public const int FlagFlying = 0x20000000;

        /// <summary>The vehicle's orientation mode while flying: the body pitched along its velocity.</summary>
        public const int FlyOrientationMode = 4;
    }
}

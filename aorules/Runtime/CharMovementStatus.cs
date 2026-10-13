namespace N3Lite.AORules
{
    /// <summary>
    /// A character's movement status and the action dispatcher over it: which movement action a mode
    /// takes (<see cref="Accepts"/>), and how each one changes the status and names the transition to
    /// run (<see cref="Apply"/>). The action numbers are the wire's <c>MovementAction</c> values.
    /// <see cref="CharMovement.Act"/> runs the transition on the body.
    /// </summary>
    public sealed class CharMovementStatus
    {
        // Modes (+4). Values as the dispatcher writes them.
        public const int ModeFrozen = 1;
        public const int ModeWalk = 2;
        public const int ModeRun = 3;
        public const int ModeSwim = 4;
        public const int ModeCrawl = 5;
        public const int ModeSneak = 6;
        public const int ModeFly = 7;
        public const int ModeSit = 8;
        public const int ModeSleep = 0xb;
        public const int ModeLounge = 0xc;

        // Axis states: 1 idle; forward/strafe 2 active; turn 4 active; jump 3 airborne.
        public const int Idle = 1;
        public const int Active = 2;
        public const int Turning = 4;
        public const int Airborne = 3;

        // Directions: forward 1 / reverse 2; left 3 / right 4; elevate up 5.
        public const int DirForward = 1;
        public const int DirReverse = 2;
        public const int DirLeft = 3;
        public const int DirRight = 4;
        public const int DirUp = 5;

        /// <summary>One status, and its defaults.</summary>
        public struct Fields
        {
            public int Mode;        // +4
            public int FwdState;    // +8
            public int FwdDir;      // +0xc
            public int StrafeState; // +0x10
            public int StrafeDir;   // +0x14
            public int ElevState;   // +0x18
            public int ElevDir;     // +0x1c
            public int TurnState;   // +0x20
            public int TurnDir;     // +0x24
            public int JumpState;   // +0x28

            public static Fields Default => new Fields
            {
                Mode = ModeRun, FwdState = Idle, StrafeState = Idle, ElevState = Idle, TurnState = Idle, JumpState = Idle,
            };

            /// <summary>Moving: driving, strafing, in a jump, or flying.</summary>
            public bool IsMoving => FwdState == Active || StrafeState == Active || JumpState == Airborne || Mode == ModeFly;
        }

        public enum Transition
        {
            None,
            ForwardStart, ForwardStop, ReverseStart, ReverseStop,
            StrafeLeftStart, StrafeRightStart, StrafeStop,
            TurnLeftStart, TurnRightStart, MouseTurnLeftStart, MouseTurnRightStart, TurnStop,
            JumpStart, JumpStop, ElevateUpStart, ElevateUpStop, FullStop,
            SwitchToFrozen, SwitchToWalk, SwitchToRun, SwitchToSwim, SwitchToCrawl, SwitchToSneak, SwitchToFly,
            SwitchToSitGround, SwitchToSleep, SwitchToLounge,
            LeaveSwim, LeaveSneak, LeaveSit, LeaveFrozen, LeaveFly, LeaveCrawl, LeaveSleep, LeaveLounge,
        }

        public Fields Current = Fields.Default;

        /// <summary>
        /// Whether the current mode takes <paramref name="action"/> from the user; one it refuses is
        /// dropped before it is applied or sent. Frozen (a root, a stun) takes only the turns, 0x10 and
        /// LeaveFrozen; anything not listed for a mode is taken, as is every action in a mode without a
        /// case.
        /// </summary>
        public bool Accepts(int action)
        {
            int a = action;
            switch (Current.Mode)
            {
                case ModeFrozen:
                    if (a < 0x1a)
                        return a == 9 || a == 0xb || a == 0xc || a == 0xe || a == 0x10;
                    return a == 0x26;
                case ModeWalk:
                    if (a == 0x18 || a == 0x21 || a == 0x22)
                        return false;
                    if (a == 0x1e)
                        return !(Current.FwdState == Active || Current.JumpState == Airborne);
                    return a != 0x25;
                case ModeRun:
                    if (a == 0x19 || a == 0x21 || a == 0x22)
                        return false;
                    if (a == 0x1e)
                        return !(Current.FwdState == Active || Current.JumpState == Airborne);
                    return true;
                case ModeSwim:
                    if (a > 0x1c)
                        return !(a == 0x1d || a == 0x1e || a == 0x1f || a == 0x24 || a == 0x27 || a == 0x28);
                    return !(a == 0x1c || a == 5 || a == 7 || a == 0xf || a == 0x18 || a == 0x19 || a == 0x1b);
                case ModeCrawl:
                    if (a < 0x1a)
                        return !(a == 0x18 || a == 0x19 || a == 5 || a == 7 || a == 0xf);
                    if (a < 0x1c)
                        return true;
                    if (a < 0x1f)
                        return false;
                    return a != 0x27;
                case ModeSneak:
                    return a != 0x21 && a != 0x2a;
                case ModeFly:
                    if (a > 0x22)
                        return a != 0x25 && !(a >= 0x28 && a <= 0x2a);
                    return !(a == 0x21 || a == 0x22 || a == 0x1b || a == 0x1c || a == 0x1e);
                case ModeSit:
                    if (a > 0x19)
                    {
                        if (a == 0x1a)
                            return true;
                        if (a == 0x21 || a == 0x22)
                            return !Current.IsMoving;
                        return a == 0x25;
                    }
                    return (a >= 9 && a <= 0xe);
                case ModeSleep:
                    return a == 0x1a || a == 0x22 || a == 0x29;
                case ModeLounge:
                    return a == 0x1a || a == 0x21 || a == 0x2a;
                default:
                    return true;
            }
        }

        /// <summary>The gait the Leave actions return to; Walk to begin with.</summary>
        public int LastSpeedMode = ModeWalk;

        /// <summary>
        /// Applies a movement action. Returns the transition to run and the status before it;
        /// <see cref="Current"/> holds the new one. <paramref name="canMove"/> is stat 224 bit 4 (see
        /// <see cref="CharMovementRules.CanFollow"/>). Does not check <see cref="Accepts"/>.
        /// </summary>
        public Transition Apply(int action, bool canMove, out Fields before)
        {
            before = Current;
            Fields cur = Current;
            Fields n = cur;
            // The new elevate state is seeded from the strafe state, as stock does.
            n.ElevState = cur.StrafeState;

            Transition t = Transition.None;
            switch (action)
            {
                case 1: // ForwardStart
                    if (cur.FwdState == Idle)
                    {
                        n.FwdState = Active;
                        n.FwdDir = DirForward;
                        t = Transition.ForwardStart;
                    }
                    if (cur.FwdState == Active && cur.FwdDir == DirReverse)
                    {
                        n.FwdState = Active;
                        n.FwdDir = DirForward;
                        t = Transition.ForwardStart;
                    }
                    break;
                case 2: // ForwardStop
                    if (cur.FwdState == Active && cur.FwdDir == DirForward)
                    {
                        n.FwdDir = 0;
                        n.FwdState = Idle;
                        t = Transition.ForwardStop;
                    }
                    break;
                case 3: // BackwardStart
                    if (cur.FwdState == Idle || (cur.FwdState == Active && cur.FwdDir == DirForward))
                    {
                        n.FwdState = Active;
                        n.FwdDir = DirReverse;
                        t = Transition.ReverseStart;
                    }
                    break;
                case 4: // BackwardStop
                    if (cur.FwdState == Active && cur.FwdDir == DirReverse)
                    {
                        n.FwdDir = 0;
                        n.FwdState = Idle;
                        t = Transition.ReverseStop;
                    }
                    break;
                case 5: // StrafeRightStart
                    if (cur.StrafeState == Idle || (cur.StrafeState == Active && cur.StrafeDir == DirLeft))
                    {
                        n.StrafeState = Active;
                        n.StrafeDir = DirRight;
                        t = Transition.StrafeRightStart;
                    }
                    break;
                case 6: // StrafeRightStop
                    if (cur.StrafeState == Active && cur.StrafeDir == DirRight)
                    {
                        n.StrafeDir = 0;
                        n.StrafeState = Idle;
                        t = Transition.StrafeStop;
                    }
                    break;
                case 7: // StrafeLeftStart
                    if (cur.StrafeState == Idle || (cur.StrafeState == Active && cur.StrafeDir == DirRight))
                    {
                        n.StrafeState = Active;
                        n.StrafeDir = DirLeft;
                        t = Transition.StrafeLeftStart;
                    }
                    break;
                case 8: // StrafeLeftStop
                    if (cur.StrafeState == Active && cur.StrafeDir == DirLeft)
                    {
                        n.StrafeDir = 0;
                        n.StrafeState = Idle;
                        t = Transition.StrafeStop;
                    }
                    break;
                case 9: // TurnRightStart
                    n.TurnState = Turning;
                    n.TurnDir = DirRight;
                    t = Transition.TurnRightStart;
                    break;
                case 10: // TurnRightMouse
                    n.TurnState = Turning;
                    n.TurnDir = DirRight;
                    t = Transition.MouseTurnRightStart;
                    break;
                case 0xb: // TurnRightStop
                    if (cur.TurnState == Turning && cur.TurnDir == DirRight)
                    {
                        n.TurnDir = 0;
                        n.TurnState = Idle;
                        t = Transition.TurnStop;
                    }
                    break;
                case 0xc: // TurnLeftStart
                    n.TurnState = Turning;
                    n.TurnDir = DirLeft;
                    t = Transition.TurnLeftStart;
                    break;
                case 0xd: // TurnLeftMouse
                    n.TurnState = Turning;
                    n.TurnDir = DirLeft;
                    t = Transition.MouseTurnLeftStart;
                    break;
                case 0xe: // TurnLeftStop
                    if (cur.TurnState == Turning && cur.TurnDir == DirLeft)
                    {
                        n.TurnDir = 0;
                        n.TurnState = Idle;
                        t = Transition.TurnStop;
                    }
                    break;
                case 0xf: // JumpStart
                    if (cur.JumpState == Airborne && cur.Mode != ModeFly)
                        break;
                    n.JumpState = Airborne;
                    t = Transition.JumpStart;
                    break;
                case 0x10: // JumpStop
                    if (cur.JumpState != Airborne)
                        break;
                    if (cur.Mode != ModeFly)
                        n.JumpState = Idle;
                    t = Transition.JumpStop;
                    break;
                case 0x11: // ElevateUpStart
                    if (cur.Mode != ModeFly)
                        break;
                    n.ElevState = Active;
                    n.ElevDir = DirUp;
                    t = Transition.ElevateUpStart;
                    break;
                case 0x12: // ElevateUpStop
                    if (cur.ElevState != Active || cur.ElevDir != DirUp)
                        break;
                    n.ElevDir = 0;
                    n.ElevState = Idle;
                    t = Transition.ElevateUpStop;
                    break;
                case 0x15: // FullStop
                    n.FwdState = Idle; n.FwdDir = 0;
                    n.StrafeState = Idle; n.StrafeDir = 0;
                    n.TurnState = Idle; n.TurnDir = 0;
                    n.JumpState = Idle;
                    n.ElevState = Idle; n.ElevDir = 0;
                    t = Transition.FullStop;
                    break;
                case 0x17: // SwitchToFrozen
                    n.Mode = ModeFrozen;
                    t = Transition.SwitchToFrozen;
                    break;
                case 0x18: // SwitchToWalk
                    n.Mode = ModeWalk;
                    t = Transition.SwitchToWalk;
                    break;
                case 0x19: // SwitchToRun — sets the last gait at once
                    n.Mode = ModeRun;
                    t = Transition.SwitchToRun;
                    LastSpeedMode = ModeRun;
                    break;
                case 0x1a: // SwitchToSwim
                    n.Mode = ModeSwim;
                    t = Transition.SwitchToSwim;
                    break;
                case 0x1b: // SwitchToCrawl — the mode is written even when refused
                    n.Mode = ModeCrawl;
                    if (cur.JumpState == Idle && cur.FwdState == Idle)
                        t = Transition.SwitchToCrawl;
                    break;
                case 0x1c: // SwitchToSneak
                    n.Mode = ModeSneak;
                    t = Transition.SwitchToSneak;
                    break;
                case 0x1d: // SwitchToFly
                    n.Mode = ModeFly;
                    t = Transition.SwitchToFly;
                    break;
                case 0x1e: // SwitchToSit
                    if (cur.IsMoving || !canMove)
                        break;
                    n.Mode = ModeSit;
                    t = Transition.SwitchToSitGround;
                    break;
                case 0x21: // SwitchToSleep — the mode is written even when refused
                    n.Mode = ModeSleep;
                    if (cur.JumpState == Idle && cur.FwdState == Idle)
                        t = Transition.SwitchToSleep;
                    break;
                case 0x22: // SwitchToLounge
                    if (cur.IsMoving)
                        break;
                    n.Mode = ModeLounge;
                    if (cur.JumpState == Idle && cur.FwdState == Idle)
                        t = Transition.SwitchToLounge;
                    break;
                case 0x23: // LeaveSwim
                    n.Mode = LastSpeedMode;
                    t = Transition.LeaveSwim;
                    break;
                case 0x24: // LeaveSneak
                    n.Mode = LastSpeedMode;
                    t = Transition.LeaveSneak;
                    break;
                case 0x25: // LeaveSit
                    if (!canMove)
                        goto frozen;
                    if (cur.Mode == ModeSit)
                        n.Mode = LastSpeedMode;
                    t = Transition.LeaveSit;
                    break;
                case 0x26: // LeaveFrozen
                    n.Mode = LastSpeedMode;
                    t = Transition.LeaveFrozen;
                    break;
                case 0x27: // LeaveFly
                    n.Mode = LastSpeedMode;
                    t = Transition.LeaveFly;
                    break;
                case 0x28: // LeaveCrawl
                    if (cur.IsMoving)
                        break;
                    if (!canMove)
                        goto frozen;
                    if (cur.Mode == ModeCrawl)
                        n.Mode = LastSpeedMode;
                    t = Transition.LeaveCrawl;
                    break;
                case 0x29: // LeaveSleep
                    if (cur.IsMoving)
                        break;
                    if (!canMove)
                        goto frozen;
                    if (cur.Mode == ModeSleep)
                        n.Mode = ModeSit;
                    t = Transition.LeaveSleep;
                    break;
                case 0x2a: // LeaveLounge
                    if (cur.IsMoving)
                        break;
                    if (!canMove)
                        goto frozen;
                    if (cur.Mode == ModeLounge)
                        n.Mode = ModeSit;
                    t = Transition.LeaveLounge;
                    break;
            }

            Current = n;
            return t;

        frozen:
            n.Mode = ModeFrozen;
            Current = n;
            return Transition.SwitchToFrozen;
        }
    }
}

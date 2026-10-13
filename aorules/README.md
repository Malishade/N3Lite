# N3Lite AO Rules

The game's movement rules on top of [N3Lite](../README.md): per-state speed curves driven by the run-speed
stat, the strafe rule, which states lock the drive, and jump height from stats. Plain C#, depends on
N3Lite only, so a client and a server can run the same code and move a character the same way.

- **Unity:** add `com.malishade.n3lite.aorules` next to `com.malishade.n3lite`, from the same repository
  (`?path=/aorules`) and the same commit.
- **.NET:** the `N3Lite.AORules` NuGet package, which depends on `N3Lite` at the same version.

## The flow

```csharp
using N3Lite;
using N3Lite.AORules;

// 1. Spawn. isNpc picks the player vehicle (four input axes) or the NPC one (follows a path).
var move = new CharMovement(isNpc: false);          // mass 50, radius 0.5 by default
move.Core.Surface = worldSurface    ;               // your ISurface; without one the body free-falls
move.Core.Warp(new Vec3(x, y, z), forward);         // spawn position and heading

// 2. Stats: full (buffed) values.
move.SetStats(new MovementStats
{
    RunSpeed = runSpeed,
    Strength = stats[16],
    Agility  = stats[17],
    GmLevel  = stats[215],
    Scale    = stats[360],     // percent; 0 counts as 100
    OrientationMode = stats[197],
    TurnSpeed = stats[267],
    MechData  = stats[662],
});

// 3. State, from the wire's MovementState number.
move.EnterState(CharMovementRules.StateRun);   // or StateWalk, StateSit, StateFly, StateRooted...
move.LeaveState();                             // back to the last Walk/Run
move.LeaveFly();                               // the LeaveFly action: back to the last Walk/Run, landing
move.ApplyStatusState(state);                  // a status read: the state as it is, no transition

// Flying, in Fly only.
move.ElevateUpStart();                         // rise at 3 m/s
move.ElevateUpStop();                          // sink at 0.8 m/s

// 4. Input.
move.Core.SetFlags(move.Core.Flags | MovementFlags.Forward);   // start moving forward
move.Core.SetFlags(move.Core.Flags & ~MovementFlags.Forward);  // stop
move.Core.TryStartJump();                                      // false if seated or already airborne

// 5. Every step.
move.Tick(dt);
Vec3 pos = move.Core.Position;
```

Call `SetStats` again, with the whole struct, whenever any of those eight stats changes. It
recomputes run speed, jump height, body height and orientation and reapplies the current state. No other stat
affects movement.

## What the rules do for you

- **Speeds** follow the state's curve: `stat / divisor + base`, clamped to the curve's range. Walk
  is a constant 1.5 m/s.
- **Strafe** is the curve at half scale, with a floor of 0.75 m/s.
- **Drive locks.** A player's vehicle refuses forward and backward drive in states 1, 8 and 9; an
  NPC's only in 1.
- **Sit** (8) refuses input and jumps, and clears the flags.
- **Fly** (7) turns gravity off. Switching to it lifts the body 0.5 m and pitches it along its
  velocity. Elevating up rises at 3 m/s; stopping sinks at 0.8 m/s. Leaving it — `LeaveFly`, or
  switching to Walk, Run or Swim — turns gravity back on, lifts the body 0.1 m, clears the
  vertical axis and sets the orientation from stat 197. Any other state taken from Fly keeps
  flying's gravity and orientation.
- **Jump height** is `(str + agi) / 200 + 1`, at least 0.5. A non-GM past 800 in total counts as
  exactly 800.
- **Body height**, which a jump keeps clear of a ceiling, is twice the body scale.
- **Orientation** comes from stat 197: 0 keeps the body level, 1 tilts it to the ground, and 2
  counts as 1. The default is 0. In Fly the body is pitched along its velocity whatever the stat.

## Driving it with movement actions

`move.Act(action)` takes the wire's `MovementAction` number through `move.Status`, the game's own
movement state machine (`CharMovementStatus`): it returns false, changing nothing, when the
current mode refuses the action (Fly refuses sitting, crawling, sneaking, sleeping, lounging, and
others; Frozen takes only the turns, the jump's end and `LeaveFrozen`), or when the controls
drop it: no jump while flying or while MechData (stat 662) is set, no sneak while sitting, and
nothing at all while `AiControlled` (a player under Fear or Mind Control) or `OnWaypointPath` is
set. Walk and run toggles are special actions (0x11 / 0x12): only when the character has that
special action available does the game send `SwitchToWalk` / `SwitchToRun`. Otherwise the status takes
it and the transition runs on the body: drive, strafe and turn into the flags, the jump, the mode
switches, and flight (`SwitchToFly` 0x1d, `ElevateUpStart` 0x11, `ElevateUpStop` 0x12,
`LeaveFly` 0x27). Elevating does nothing out of Fly.

- The jump's end (0x10) runs by itself when a jump lands, inside the landing as the game runs
  it; don't send it.
- `Act` sets the turn rate itself, from the turn speed (stat 267, `MovementStats.TurnSpeed`):
  3.5 rad/s standing and 1.5 driving at 0, otherwise `stat / 11000` standing and half that
  driving. Set when a keyboard turn starts and again when the drive starts or stops mid-turn; a
  mouse turn has none.
- A full status message goes in with `move.ApplyStatus(fields, lastSpeedMode)`, with no
  transition.
- Flying sets bit `0x20000000` (`CharMovementRules.FlagFlying`) in the Flags stat (stat 0) and
  leaving clears it. The stats are yours: handle `move.FlagsSet` and `move.FlagsCleared`.

## Keeping two sides in step

- **The same stats.** Both sides fill `MovementStats` from the same full values.
- **The same frame.** Positions and the surface are in the same coordinates on both sides.
- **The same step.** The same `dt` sequence gives the same result; a fixed step on one side and a
  variable one on the other will be close but not identical.
- **Your protocol.** Which message means which flag or state is not in this package. Each side maps
  its own messages onto `EnterState`, `LeaveState`, `LeaveFly`, `ApplyStatusState`,
  `ElevateUpStart`, `ElevateUpStop`, `Core.SetFlags` and `Core.TryStartJump`.

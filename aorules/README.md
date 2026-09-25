# N3Lite AO Rules

Stock movement rules on top of [N3Lite](../README.md): per-state speed curves driven by the run-speed
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
move.Core.Surface = playfieldSurface;               // your ISurface; without one the body free-falls
move.Core.Warp(new Vec3(x, y, z), forward);         // spawn position and heading

// 2. Stats: full (buffed) values.
move.SetStats(new MovementStats
{
    RunSpeed = runSpeed,
    Strength = stats[16],
    Agility  = stats[17],
    GmLevel  = stats[215],
    Scale    = stats[360],     // percent; 0 counts as 100
});

// 3. State, from the wire's MovementState number.
move.EnterState(CharMovementRules.StateRun);   // or StateWalk, StateSit, StateFly, StateRooted...
move.LeaveState();                             // back to the last Walk/Run

// 4. Input.
move.Core.SetFlags(move.Core.Flags | MovementFlags.Forward);   // start moving forward
move.Core.SetFlags(move.Core.Flags & ~MovementFlags.Forward);  // stop
move.Core.TryStartJump();                                      // false if seated or already airborne

// 5. Every step.
move.Tick(dt);
Vec3 pos = move.Core.Position;
```

Call `SetStats` again, with the whole struct, whenever any of those five stats changes. It
recomputes run speed, jump height and body height and reapplies the current state. No other stat
affects movement.

## What the rules do for you

- **Speeds** follow the state's curve: `stat / divisor + base`, clamped to the curve's range. Walk
  is a constant 1.5 m/s.
- **Strafe** is the curve at half scale, with a floor of 0.75 m/s.
- **Drive locks.** A player's vehicle refuses forward and backward drive in states 1, 8 and 9; an
  NPC's only in 1.
- **Sit** (8) refuses input and jumps, and clears the flags.
- **Fly** (7) turns gravity off.
- **Jump height** is `(str + agi) / 200 + 1`, at least 0.5. A non-GM past 800 in total counts as
  exactly 800.
- **Body height**, which a jump keeps clear of a ceiling, is twice the body scale.

## Keeping two sides in step

- **The same stats.** Both sides fill `MovementStats` from the same full values.
- **The same frame.** Positions and the surface are in the same coordinates on both sides.
- **The same step.** The same `dt` sequence gives the same result; a fixed step on one side and a
  variable one on the other will be close but not identical.
- **Your protocol.** Which message means which flag or state is not in this package. Each side maps
  its own messages onto `EnterState`, `LeaveState`, `Core.SetFlags` and `Core.TryStartJump`.

# N3Lite

A vehicle simulation and collision library in plain C#: characters, NPCs and cameras moved by one
sub-stepped integrator, against heightmap terrain and triangle-mesh geometry. One source tree,
usable from Unity and from plain .NET.

- **Vehicles** — a shared integrator with three steering channels (longitudinal force, lateral
  velocity, turn rate); a four-axis character vehicle with a jump; an NPC vehicle that follows
  waypoint paths; first-person and third-person camera vehicles with zoom and occlusion handling.
- **Characters** — `N3CharVehicle` wraps the character vehicle: movement flags to the four axes, a
  `MovementProfile` for its speeds and what it may do (drive, take input, fly), the jump and NPC
  paths. The game decides the numbers; one `Tick(dt)` per frame.
- **Ground contact** — a ground clamp with step height, a slope gate, a tripod ground probe and a
  swept move with wall sliding.
- **Surfaces** — a chunked heightmap walked tile by tile, a square cell grid of triangle meshes with
  a BVH per mesh, and ray and floor queries over both.

## Layout

| Path | What |
|---|---|
| `package/` | The Unity package (`com.malishade.n3lite`). `Runtime/` holds every source file and the `N3Lite` asmdef. |
| `src/N3Lite.csproj` | The same sources built for .NET, `netstandard2.1`. |
| `tests/N3Lite.Tests/` | xUnit tests, run against the `netstandard2.1` build. |

The sources have no Unity dependency. The one exception is `[BurstCompile]` on the triangle-mesh
queries (`TriangleMeshKernels`): inside Unity, Burst compiles them; outside Unity,
`Runtime/BurstStub.cs` supplies the attribute and they run as ordinary C#.

## Using it

**Unity** — add to `Packages/manifest.json` (path relative to the project's `Packages/` folder):

```json
"com.malishade.n3lite": "file:../../N3Lite/package"
```

and reference the `N3Lite` assembly from your asmdef.

**.NET** — reference the project:

```xml
<ProjectReference Include="..\N3Lite\src\N3Lite.csproj" />
```

## Build and test

```
dotnet build src/N3Lite.csproj
dotnet test N3Lite.slnx
```

## Floating point

Burst (Strict float mode) and .NET both round every step to single precision. Unity's Mono does
not — it evaluates float expressions in double precision — so with Burst off, or in code Burst does
not compile, results can differ between the two in the last bits.

## License

Public domain ([the Unlicense](LICENSE)).

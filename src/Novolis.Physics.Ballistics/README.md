<!-- novolis-pkg-brand:start -->
[![Novolis](https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.png)](https://novolis-platform.github.io/.github/novolis-physics/)

[Novolis](https://github.com/Novolis-Platform) · [Docs](https://novolis-platform.github.io/.github/novolis-physics/) · [Source](https://github.com/Novolis-Platform/novolis-physics)
<!-- novolis-pkg-brand:end -->

# Novolis.Physics.Ballistics

Projectile state, drag models, analytic queries, and terrain-aware trajectory runners.

## Install

```bash
dotnet add package Novolis.Physics.Ballistics
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Ballistics;

var sim = new ProjectileBallisticSimulation(ProjectileProfile.CannonDefaults);
var state = sim.Launch(origin, velocity);
state = sim.Step(state, dt, environment);
```

## API

| Type | Role |
|------|------|
| `ProjectileState` | Position, velocity, mass, drag state |
| `ProjectileProfile` | Mass, reference area, constant Cd |
| `ProjectileBallisticSimulation` | Step with gravity, optional quadratic drag, optional Mach Cd, optional Coriolis |
| `DragCoefficientLaw` | Piecewise-linear Cd(Mach). `BoatTail` is an in-repo educational fit |
| `CoriolisAcceleration` | East-Up-North Coriolis (`+X` east, `+Y` up, `+Z` north) |
| `StandardAtmosphere` | Moist density, barometric pressure, dry-air speed of sound |
| `BallisticTrajectoryRunner` | Full trajectory with terrain contact |
| `ProjectileTerrainStepper` | Heightfield-aware stepping |

`ProjectileBallisticEnvironment.LatitudeDegrees` turns Coriolis on when it is a finite latitude. Leave it NaN for the constant-Cd path. A Mach law replaces the profile Cd and still uses the profile mass and area. Passing no law keeps the constant Cd.

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Collision.Simple` | Mesh collision during flight |
| `Novolis.Physics.Aerodynamics` | Atmosphere and drag models |

## More documentation

- [Ballistics example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/ballistics.md)


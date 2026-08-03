<!-- novolis-pkg-brand:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform/novolis-physics">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.svg" width="72" alt="Novolis"/>
  </a>
</p>
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
| `ProjectileProfile` | Mass, drag area, `CannonDefaults` preset |
| `ProjectileBallisticSimulation` | Launch and step projectile |
| `BallisticTrajectoryRunner` | Full trajectory with terrain contact |
| `BallisticsQueries` | Range, apex, time-of-flight helpers |
| `ProjectileTerrainStepper` | Heightfield-aware stepping |
| `StandardAtmosphere` | ISA density model |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Collision.Simple` | Mesh collision during flight |
| `Novolis.Physics.Aerodynamics` | Atmosphere and drag models |

## More documentation

- [Ballistics example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/ballistics.md)


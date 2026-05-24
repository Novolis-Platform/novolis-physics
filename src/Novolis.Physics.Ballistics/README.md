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

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Collision.Simple` | Mesh collision during flight |
| `Novolis.Physics.Aerodynamics` | Atmosphere and drag models |

## More documentation

- [Ballistics example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/ballistics.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

# Novolis.Physics.Gravity

Point-mass and patched-conic gravity models as `IForceModel` implementations.

## Install

```bash
dotnet add package Novolis.Physics.Gravity
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Gravity;

var field = new PointMassField(new Vector3(0, 0, 0), mu: 3.986004418e14);
var gravity = new PointMassGravityModel(field);
```

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Motion` | Integrate with `SimulationPipeline` |
| `Novolis.Physics.Orbits` | Dedicated leapfrog orbit stack (no pipeline) |

## More documentation

- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

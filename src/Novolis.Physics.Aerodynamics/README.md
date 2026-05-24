# Novolis.Physics.Aerodynamics

Atmosphere density models and simple lift/drag forces for projectiles and rigid bodies.

## Install

```bash
dotnet add package Novolis.Physics.Aerodynamics
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Aerodynamics;

IAtmosphereModel atmosphere = new ExponentialAtmosphereModel(seaLevelDensity: 1.225, scaleHeight: 8500);
```

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Ballistics` | Projectile drag and trajectory helpers |
| `Novolis.Physics.Abstractions` | `IForceModel` integration |

## More documentation

- [Ballistics example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/ballistics.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

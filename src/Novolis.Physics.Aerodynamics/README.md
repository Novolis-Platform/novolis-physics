<!-- novolis-pkg-brand:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform/novolis-physics">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.svg" width="72" alt="Novolis"/>
  </a>
</p>
<!-- novolis-pkg-brand:end -->

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
var env = new SimpleAeroEnvironment(atmosphere, altitudeMeters: 100, windWorld: Vector3.Zero,
    referenceAreaM2: 0.1, dragCoefficient: 0.02, liftCoefficient: 0.4,
    liftReferenceForwardWorld: Vector3.UnitZ);
var aero = new SimpleLiftDragModel();
```

## API

| Type | Role |
|------|------|
| `IAtmosphereModel` | Density at altitude |
| `ExponentialAtmosphereModel` | Exponential scale-height atmosphere |
| `SimpleAeroEnvironment` | Bundled atmosphere + wind |
| `SimpleLiftDragModel` | Lift/drag `IForceModel` from coefficients |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Ballistics` | Projectile drag and trajectory helpers |
| `Novolis.Physics.Abstractions` | `IForceModel` integration |

## More documentation

- [Ballistics example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/ballistics.md)


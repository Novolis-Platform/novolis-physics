# Novolis.Physics.Orbits

Central-body orbit math and leapfrog SoA integration (parallel to the force-first pipeline).

## Install

```bash
dotnet add package Novolis.Physics.Orbits
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Orbits;

var sim = new LeapfrogCentralBodySoA(mu: 3.986e14, bodyCount: 1);
sim.SetState(0, position, velocity);
sim.Step(dt, KernelMode.Scalar);
```

## API

| Type | Role |
|------|------|
| `LeapfrogCentralBodySoA` | SoA leapfrog integrator for N bodies |
| `CentralOrbitSimulator` | Single-body central orbit helper |
| `OrbitalMath` | Kepler elements, vis-viva, anomaly conversions |
| `OrbitState` | Semi-major axis, eccentricity, true anomaly |
| `KernelMode` | `Scalar` vs vectorized stepping |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Gravity` | `IForceModel` gravity in the pipeline |
| `Novolis.Physics.Astro` | Convert catalog ly to SI meters |

## More documentation

- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md)

# Novolis.Physics.Abstractions

Core contracts for force-first physics: forces, integrators, rigid-body state, and static-world queries.

## Install

```bash
dotnet add package Novolis.Physics.Abstractions
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Abstractions;
using Novolis.Physics.Motion;

IIntegrator<RigidBodyState> integrator = new SemiImplicitEulerRigidBodyIntegrator();
// Sum ForceSample values from IForceModel implementations, then integrator.Step(...)
```

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Motion` | `SimulationPipeline`, fixed-step helper |
| `Novolis.Physics` | Meta-package for the full stack |

## More documentation

- [Getting started](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/getting-started.md)
- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

<!-- novolis-pkg-brand:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform/novolis-physics">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.svg" width="72" alt="Novolis"/>
  </a>
</p>
<!-- novolis-pkg-brand:end -->

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

## API

| Type | Role |
|------|------|
| `RigidBodyState` | Position, velocity, orientation, angular velocity |
| `ForceSample` | Linear and torque contribution |
| `IForceModel<TState>` | `Evaluate(state, environment)` → forces |
| `IIntegrator<TState>` | `Step(state, forces, dt)` |
| `IStaticWorld` | Raycast and overlap queries |
| `HitInfo` | Ray hit distance, normal, triangle index |
| `IHeightSampler` | Terrain height sampling |
| `IProjectileTerrainContact` | Projectile ground contact hook |
| `AxisAlignedRangeBox` | Spatial query bounds |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Motion` | `SimulationPipeline`, fixed-step helper |
| `Novolis.Physics` | Meta-package for the full stack |

## More documentation

- [Getting started](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/getting-started.md)
- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md)


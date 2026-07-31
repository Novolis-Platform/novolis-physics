# Novolis.Physics

Installs the full Novolis physics stack in one reference.

## Install

```bash
dotnet add package Novolis.Physics
```

## Quick start

```csharp
using Novolis.Physics.Abstractions;
using Novolis.Physics.Motion;

var pipeline = new SimulationPipeline<RigidBodyState>(
    forceModels: [/* IForceModel<RigidBodyState> */],
    integrator: new SemiImplicitEulerRigidBodyIntegrator());
```

This meta-package pulls in Abstractions, Motion, Gravity, Aerodynamics, Collision.Simple, Joints, Ballistics, Orbits, and Astro. Reference individual packages when you need a subset.

## API

| Package | Role |
|---------|------|
| `Novolis.Physics.Abstractions` | Forces, integrators, rigid-body state |
| `Novolis.Physics.Motion` | `SimulationPipeline`, fixed-step helper |
| `Novolis.Physics.Gravity` | Point-mass and patched-conic gravity |
| `Novolis.Physics.Aerodynamics` | Atmosphere and lift/drag |
| `Novolis.Physics.Ballistics` | Projectile trajectories |
| `Novolis.Physics.Collision.Simple` | BVH mesh queries, sphere piles |
| `Novolis.Physics.Joints` | Distance joints, ragdoll presets |
| `Novolis.Physics.Orbits` | Leapfrog central-body integration |
| `Novolis.Physics.Astro` | ly/pc/AU ↔ meters |

## Related

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md) — which API to use
- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md) — packages and integration styles
- [Examples](https://github.com/Novolis-Platform/novolis-physics/tree/main/docs/examples) — ballistics, collision, DI

## Support

Pre-release platform library. All packable packages ship strict XML API documentation (`CS1591` enforced).

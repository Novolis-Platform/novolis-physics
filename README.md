<!-- novolis-marketing:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-brand-transparent.svg" width="360" alt="Novolis"/>
  </a>
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/banners/novolis-physics.svg" width="100%" alt="novolis-physics"/>
</p>

<p align="center">
  <strong>Force-first textbook physics</strong><br/>
  Motion, gravity, ballistics, cloth, collision, orbits — Math only, no cameras.
</p>

<p align="center">
  <a href="https://novolis-platform.github.io/.github/novolis-physics/"><img src="https://img.shields.io/badge/docs-portfolio-0a7ea3" alt="docs"/></a>
  <a href="https://github.com/Novolis-Platform/novolis-physics/actions"><img src="https://img.shields.io/github/actions/workflow/status/Novolis-Platform/novolis-physics/merge.yml?branch=main&label=merge&logo=github" alt="merge"/></a>
  <a href="https://github.com/orgs/Novolis-Platform/packages?repo_name=novolis-physics"><img src="https://img.shields.io/badge/packages-GitHub%20Packages-0a7ea3?logo=nuget" alt="packages"/></a>
  <a href="https://github.com/Novolis-Platform"><img src="https://img.shields.io/badge/org-Novolis--Platform-111827" alt="org"/></a>
</p>

<p align="center">
  <a href="https://novolis-platform.github.io/.github/novolis-physics/">Docs</a>
  ·
  <a href="https://nuget.pkg.github.com/Novolis-Platform/index.json"><code>https://nuget.pkg.github.com/Novolis-Platform/index.json</code></a>
  ·
  <a href="https://github.com/Novolis-Platform/.github/blob/main/profile/README.md">Org landing</a>
  ·
  <a href="https://github.com/Novolis-Platform/novolis-governance">Governance</a>
</p>

---
<!-- novolis-marketing:end -->
<!-- novolis-package-index:start -->
> **GitHub Packages shows this repository README on every package page** (upstream limitation).
> Open the **package README** for install and quick start — embedded in each .nupkg and linked below.

## Published packages

| Package | Install | Package README |
|---------|---------|----------------|
| `Novolis.Physics` | `dotnet add package Novolis.Physics` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics/README.md) |
| `Novolis.Physics.Abstractions` | `dotnet add package Novolis.Physics.Abstractions` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Abstractions/README.md) |
| `Novolis.Physics.Aerodynamics` | `dotnet add package Novolis.Physics.Aerodynamics` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Aerodynamics/README.md) |
| `Novolis.Physics.Astro` | `dotnet add package Novolis.Physics.Astro` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Astro/README.md) |
| `Novolis.Physics.Ballistics` | `dotnet add package Novolis.Physics.Ballistics` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Ballistics/README.md) |
| `Novolis.Physics.Cloth` | `dotnet add package Novolis.Physics.Cloth` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Cloth/README.md) |
| `Novolis.Physics.Collision.Simple` | `dotnet add package Novolis.Physics.Collision.Simple` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Collision.Simple/README.md) |
| `Novolis.Physics.Gravity` | `dotnet add package Novolis.Physics.Gravity` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Gravity/README.md) |
| `Novolis.Physics.Joints` | `dotnet add package Novolis.Physics.Joints` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Joints/README.md) |
| `Novolis.Physics.Motion` | `dotnet add package Novolis.Physics.Motion` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Motion/README.md) |
| `Novolis.Physics.Orbits` | `dotnet add package Novolis.Physics.Orbits` | [README](https://github.com/Novolis-Platform/novolis-physics/blob/main/src/Novolis.Physics.Orbits/README.md) |

For NuGet.org and Visual Studio, the **embedded** README.md inside each package is authoritative.

<!-- novolis-package-index:end -->
# Novolis.Physics

Force-first **textbook** physics for .NET — numerics, motion pipeline, gravity, ballistics, collision, and orbits.

**Scope:** forces, integration, collision **response**, and domain solvers (ballistics, orbits, gravity, aero). Depends on `Novolis.Math.*` only. **Not** cameras, players, AI, rendering, ECS, or simulation orchestration — see [`novolis-simulation`](../novolis-simulation) and [library boundaries](../novolis-governance/docs/library-boundaries.md).

**Documentation:** [docs/README.md](docs/README.md) (integration guide, architecture, examples).

## Install

```bash
dotnet add package Novolis.Physics
```

Or reference individual packages (`Novolis.Physics.Motion`, `Novolis.Physics.Orbits`, …).

## Quick start

+Y is up; gravity for ballistics uses **−Y**. Build a pipeline, then step each fixed timestep:

```csharp
using Novolis.Physics.Abstractions;
using Novolis.Physics.Gravity;
using Novolis.Physics.Motion;
using System.Numerics;

var integrator = new SemiImplicitEulerRigidBodyIntegrator();
var gravity = new PointMassGravityModel();
var pipeline = new SimulationPipeline<RigidBodyState, PointMassField>(integrator, gravity);

var field = new PointMassField([(Vector3.Zero, 3.986e14f)]);
var body = new RigidBodyState(
    new Vector3(6_771_000f, 0, 0),
    new Vector3(0, 7_500f, 0),
    Quaternion.Identity,
    Vector3.Zero,
    mass: 1.0,
    inertiaDiagonalBody: new Vector3(1, 1, 1));

var acc = new FixedStepAccumulator(1.0 / 60.0);
double time = 0;
acc.AddTimeAndDrain(1.0 / 30.0, dt =>
{
    body = pipeline.Step(body, field, dt, time);
    time += dt;
});
```

See [docs/INTEGRATION.md](docs/INTEGRATION.md) for ballistics (facade vs pipeline), collision, and orbits. Copy-paste recipes: [docs/examples/](docs/examples/). Architecture overview: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md). Versioning: [docs/VERSIONING.md](docs/VERSIONING.md).

## Integration styles

| Goal | Start with |
|------|------------|
| Rigid body + custom forces | `SimulationPipeline` + `SemiImplicitEulerRigidBodyIntegrator` |
| Cannon / drag prototype | `ProjectileBallisticSimulation` |
| Sphere in a static mesh | `BvhStaticSphereIntegrator` + `BvhStaticWorld` |
| Two-body orbit tests | `CentralOrbitSimulator` (separate from the pipeline) |

## Conventions

- Right-handed 3D coordinates; **+Y is up**.
- Gravity and ballistics use **−Y** for uniform gravity.
- Planar cannon problems often use +X range with `Z = 0`.

## Packages

| Package | Role |
|---------|------|
| `Novolis.Physics` | Aggregate — all product packages |
| `Novolis.Physics.Abstractions` | Force models, integrators, static-world queries |
| `Novolis.Physics.Motion` | Rigid-body motion pipeline |
| `Novolis.Physics.Gravity` | Point / patched-conic gravity |
| `Novolis.Physics.Aerodynamics` | Lift / drag models |
| `Novolis.Physics.Collision.Simple` | Static mesh BVH queries |
| `Novolis.Physics.Ballistics` | Projectile drag and sweeps |
| `Novolis.Physics.Orbits` | Two-body orbital helpers |

## Build

```bash
dotnet build Novolis.Physics.slnx
dotnet run --project tests/Novolis.Physics.Unit -c Release
```


# Getting started

**novolis-physics** provides modular NuGet packages for force-first rigid-body simulation, ballistics, simple collision, joints/ragdolls, and orbital mechanics.

## Prerequisites

- [.NET 10 SDK](https://dotnet.microsoft.com/download)

## Install

Full stack (meta-package):

```bash
dotnet add package Novolis.Physics
```

Or install individual packages — see each `src/*/README.md` and [ARCHITECTURE.md](ARCHITECTURE.md).

## Quick start

```csharp
using Novolis.Physics.Motion;
using Novolis.Physics.Gravity;
using Novolis.Physics.Abstractions;

var pipeline = new SimulationPipeline<RigidBodyState>(
    [new PointMassGravityModel(field)],
    new SemiImplicitEulerRigidBodyIntegrator());
```

## More documentation

- [INTEGRATION.md](INTEGRATION.md) — which API style to use
- [examples/](examples/) — ballistics, collision, DI


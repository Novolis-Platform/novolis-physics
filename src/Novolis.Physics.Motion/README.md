# Novolis.Physics.Motion

Rigid-body integration, quaternion angular kinematics, and the canonical force-first `SimulationPipeline`.

## Install

```bash
dotnet add package Novolis.Physics.Motion
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Abstractions;
using Novolis.Physics.Motion;

var pipeline = new SimulationPipeline<RigidBodyState>(
    forceModels: [/* IForceModel<RigidBodyState> */],
    integrator: new SemiImplicitEulerRigidBodyIntegrator());
body = pipeline.Step(body, environment, dtSeconds);
```

## API

| Type | Role |
|------|------|
| `SimulationPipeline<TState>` | Sum force models + integrate one step |
| `SemiImplicitEulerRigidBodyIntegrator` | Default rigid-body integrator |
| `QuaternionAngularIntegration` | Angular velocity → orientation |
| `FixedStepAccumulator` | Fixed-timestep sub-stepping |
| `UniformAccelerationEnergy` | Kinetic/potential energy helpers |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Abstractions` | `IForceModel`, `IIntegrator`, `RigidBodyState` |
| `Novolis.Physics.Gravity` | Gravity `IForceModel` implementations |

## More documentation

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md)

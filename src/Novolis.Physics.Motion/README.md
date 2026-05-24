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

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Abstractions` | `IForceModel`, `IIntegrator`, `RigidBodyState` |
| `Novolis.Physics.Gravity` | Gravity `IForceModel` implementations |

## More documentation

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

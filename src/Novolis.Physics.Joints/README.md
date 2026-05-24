# Novolis.Physics.Joints

Distance joints, swing/hinge limits, ragdoll presets, and constrained sphere simulators.

## Install

```bash
dotnet add package Novolis.Physics.Joints
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Joints;

var simulator = new ConstrainedSphereSimulator();
RagdollHumanoidPreset.BuildStanding(groundPoint, spheres, joints, swings, hinges);
simulator.Step(staticWorld, spheres, interior, dt, swings, hinges);
```

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Collision.Simple` | `SphereInStaticWorldSimulator`, BVH worlds |
| `Novolis.Physics` | Full physics stack meta-package |

## More documentation

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md)

## Support

Pre-release platform library. Public API is fully documented with strict XML (`CS1591` enforced).

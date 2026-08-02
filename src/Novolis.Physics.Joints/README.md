# Novolis.Physics.Joints

Distance joints, swing/hinge limits, ragdoll presets, and constrained sphere simulators.

For fabric / particle sheets, use **`Novolis.Physics.Cloth`** (separate package).

For **skeletal reach / FK / IK** (hand or foot targets), use **`Novolis.Simulation.Humanoid`** — joint constraint projection here is dynamics, not inverse kinematics.

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

## API

| Type | Role |
|------|------|
| `DistanceJoint` | Two-sphere length constraint |
| `SwingLimit` | Cone swing limit between bones |
| `HingeLimit` | Hinge axis limit |
| `DistanceJointSolver` | Satisfy distance constraints (ragdoll-tuned defaults) |
| `AngularLimitSolver` | Apply swing/hinge limits |
| `ConstrainedSphereSimulator` | Joint + contact step |
| `RagdollHumanoidPreset` | `BuildStanding`, humanoid sphere layout |
| `RagdollBodyCollision` | Self-collision between ragdoll parts |
| `BoneFrame` | Local bone orientation helper |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Cloth` | Flags, draping, cloth cutting |
| `Novolis.Simulation.Humanoid` | Skeletal FK / IK (not joint dynamics) |
| `Novolis.Physics.Collision.Simple` | `SphereInStaticWorldSimulator`, BVH worlds over Math `TriangleMesh` |
| `Novolis.Simulation.Humanoid.Physics` | Bridge to humanoid bind poses |

## More documentation

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md)

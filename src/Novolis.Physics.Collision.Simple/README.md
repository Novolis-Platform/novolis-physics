# Novolis.Physics.Collision.Simple

Static mesh BVH queries, sphere piles, and swept-sphere integration against triangle worlds.

## Install

```bash
dotnet add package Novolis.Physics.Collision.Simple
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Collision.Simple;

IStaticWorld world = new BvhStaticWorld(mesh);
var integrator = new BvhStaticSphereIntegrator(world);
```

## API

| Type | Role |
|------|------|
| `BvhStaticWorld` | Triangle mesh BVH implementing `IStaticWorld` |
| `EmptyStaticWorld` | No-op static world |
| `StaticTriangleMesh` | Indexed triangle storage |
| `SphereState` | Position, radius, velocity |
| `SphereSoA` | Structure-of-arrays sphere batch |
| `BvhStaticSphereIntegrator` | Swept-sphere against BVH |
| `SphereInStaticWorldSimulator` | Pile simulation with contact resolution |
| `UniformGridSphereContactSolver` | Broad-phase sphere contacts |
| `InteriorClampVolume` | Keep spheres inside bounds |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Ballistics` | Projectile mesh sweeps |
| `Novolis.Physics.Joints` | Ragdoll sphere piles with joints |

## More documentation

- [Collision room example](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/examples/collision-room.md)

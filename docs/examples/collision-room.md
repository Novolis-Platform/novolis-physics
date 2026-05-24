# Collision room example

`Novolis.Physics.Collision.Simple` is **query-oriented**: static meshes, raycasts, and approximate sphere sweeps. Full rigid-body contact is not a separate pipeline stage — use `BvhStaticSphereIntegrator` for a bouncing sphere.

## Build a static world

```csharp
using System.Numerics;
using Novolis.Physics.Collision.Simple;

// Floor quad (two triangles) in the XZ plane at y = 0.
var verts = new[]
{
    new Vector3(-50, 0, -50),
    new Vector3(50, 0, -50),
    new Vector3(50, 0, 50),
    new Vector3(-50, 0, 50),
};
var indices = new[] { 0, 1, 2, 0, 2, 3 };
var mesh = new StaticTriangleMesh(verts, indices);
var world = new BvhStaticWorld(mesh);
```

## Advance a sphere with gravity and contact

```csharp
using System.Numerics;

var center = new Vector3(0, 5, 0);
var velocity = Vector3.Zero;
const double radius = 0.5;
var gravity = new Vector3(0, -9.81f, 0);

for (var step = 0; step < 300; step++)
{
    velocity += gravity * (1.0f / 60f);
    BvhStaticSphereIntegrator.AdvanceOneStep(
        world,
        ref center,
        ref velocity,
        radius,
        dtSeconds: 1.0 / 60.0,
        normalRestitution: 0.6);
}
```

`AdvanceWithUniformAccelerationAndLinearDrag` applies gravity and isotropic linear drag per sub-step, then sweeps. Contact uses `SphereContactKinematics.ReflectWithRestitution` internally.

## Sweeps vs integration

| API | Role |
|-----|------|
| `IStaticWorld.SweepSphere` | Predict hits along a displacement (approximate CCD) |
| `BvhStaticSphereIntegrator` | Integrate position with contact resolution |

Fast motion or thin geometry can miss sweeps; use smaller `dt` or read [INTEGRATION.md](../INTEGRATION.md) §3.

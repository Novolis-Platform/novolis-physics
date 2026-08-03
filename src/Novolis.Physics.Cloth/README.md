<!-- novolis-pkg-brand:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform/novolis-physics">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.svg" width="72" alt="Novolis"/>
  </a>
</p>
<!-- novolis-pkg-brand:end -->

# Novolis.Physics.Cloth

Fabric / particle-sheet simulation: hanging flags, draping, strain limits, blade and blast cutting.

This package is **separate from** `Novolis.Physics.Joints` (ragdolls / angular limits). Cloth reuses `DistanceJoint` + `DistanceJointSolver` as shared length constraints, then adds fabric-first stepping (`MaxStretchRatio`, wind, pins) and topology cutting.

## Install

```bash
dotnet add package Novolis.Physics.Cloth
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

```csharp
using Novolis.Physics.Cloth;
using Novolis.Physics.Joints;

var options = new ClothSheetOptions { Columns = 14, Rows = 10, PinMode = ClothPinMode.TopRow };
ClothSheetPreset.BuildHanging(origin, Vector3.UnitX, -Vector3.UnitY, options, spheres, joints, pins, anchors);

var cloth = new ClothSheetSimulator
{
    MaxStretchRatio = 1.06f,
    WindAcceleration = new Vector3(2f, 0f, 0.5f),
};
cloth.SetJoints(joints);
cloth.SetPins(pins, anchors);
cloth.Step(staticWorld, spheres, interior, dt);

ClothCutOps.CutWithBlade(joints, spheres, new ClothBlade(heel, tip, halfThickness: 0.06f));
cloth.SetJoints(joints);
```

## API

| Type | Role |
|------|------|
| `ClothSheetOptions` / `ClothPinMode` | Grid size, stiffness, pin mode |
| `ClothSheetPreset` | Build hanging / drop sheet |
| `ClothSheetSimulator` | Integrate + project + strain clamp + wind/pins |
| `ClothBlade` / `ClothBlast` / `ClothCutResult` | Cut queries |
| `ClothCutOps` | Sever / blade / blast / impulse |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Physics.Joints` | Ragdolls, swing/hinge, `DistanceJoint` |
| `Novolis.Physics.Collision.Simple` | Sphere world, BVH static meshes |

## More documentation

- [Integration guide](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/INTEGRATION.md)


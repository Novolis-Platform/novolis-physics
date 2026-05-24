# Novolis.Physics.Numerics

**Obsolete** compatibility shims (`Vector3d`, `Rayd`, …). New code should use `System.Numerics` and `Novolis.Math.Geometry`.

## Install

```bash
dotnet add package Novolis.Physics.Numerics
```

**Prerequisites:** [.NET 10 SDK](https://dotnet.microsoft.com/download) (`net10.0`).

## Quick start

Prefer BCL types instead of this package:

```csharp
using System.Numerics;
using Novolis.Math.Geometry;
// Use Vector3, Quaternion, Ray — not Vector3d / Rayd
```

## Related packages

| Package | When to use |
|---------|-------------|
| `Novolis.Math.Geometry` | `Ray`, `Sphere3`, meshes |
| `Novolis.Physics.Abstractions` | Current physics contracts |

## More documentation

- [Architecture](https://github.com/Novolis-Platform/novolis-physics/blob/main/docs/ARCHITECTURE.md)

## Support

Deprecated surface; retained for migration only. Public API is documented with strict XML (`CS1591` enforced).

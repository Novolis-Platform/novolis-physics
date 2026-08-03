<!-- novolis-pkg-brand:start -->
<p align="center">
  <a href="https://github.com/Novolis-Platform/novolis-physics">
    <img src="https://raw.githubusercontent.com/Novolis-Platform/.github/main/brand/logo-icon.svg" width="72" alt="Novolis"/>
  </a>
</p>
<!-- novolis-pkg-brand:end -->

# Novolis.Physics.Astro

Astronomical unit conversions (ly / pc / AU ↔ meters) for bridging stellar catalogs to SI physics.

## Install

```bash
dotnet add package Novolis.Physics.Astro
```

## Quick start

```csharp
using Novolis.Physics.Astro;

var meters = AstronomicalUnits.LyToMeters(4.37);
var pos = AstronomicalUnits.LightYearsToVector3(4.37, 0, 0);
```

## API

| Member | Role |
|--------|------|
| `MetersPerLy`, `MetersPerPc`, `MetersPerAu` | Conversion constants |
| `LyToMeters`, `MetersToLy` | Light-year ↔ meters |
| `PcToMeters`, `MetersToPc` | Parsec ↔ meters |
| `AuToMeters`, `MetersToAu` | Astronomical unit ↔ meters |
| `LightYearsToVector3(xLy, yLy, zLy)` | Catalog coords → `Vector3` meters |

## Related

| Package | When to use |
|---------|-------------|
| `Novolis.Astro.Abstractions` | Stellar coordinates in light-years |
| `Novolis.Physics.Motion` | Integrate bodies in SI units |


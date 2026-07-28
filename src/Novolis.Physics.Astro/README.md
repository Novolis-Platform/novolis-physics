# Novolis.Physics.Astro

Astronomical unit conversions (ly / pc / AU ↔ meters) for bridging catalogs to SI physics.

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

# Release

This repository publishes with the org CalVer scheme (`2026.1.*`) via `merge.yml` to GitHub Packages when packages are packable.

See [release-policy](https://github.com/Novolis-Platform/novolis-governance/blob/main/docs/release-policy.md).

Published docs: [https://novolis-platform.github.io/.github/novolis-physics/](https://novolis-platform.github.io/.github/novolis-physics/)

## Packages

- `Novolis.Physics`
- `Novolis.Physics.Abstractions`
- `Novolis.Physics.Aerodynamics`
- `Novolis.Physics.Astro`
- `Novolis.Physics.Ballistics`
- `Novolis.Physics.Cloth`
- `Novolis.Physics.Collision.Simple`
- `Novolis.Physics.Gravity`
- `Novolis.Physics.Joints`
- `Novolis.Physics.Motion`
- `Novolis.Physics.Orbits`

## Consumers

Restore from nuget.org + `https://nuget.pkg.github.com/Novolis-Platform/index.json` only.

Local multi-repo iteration: open `d:\novolis\Novolis.Platform.slnx` (ProjectReference mode) — do not add a local feed.

# Design

Motion, gravity, ballistics, cloth, collision, orbits — Math only, no cameras.

Published docs: [https://novolis-platform.github.io/.github/novolis-physics/](https://novolis-platform.github.io/.github/novolis-physics/)

## Layer placement

Closed spine: **Physics** over Math. No cameras, Raylib, or Avalonia.

## Goals

- Keep public APIs documented and packable as `Novolis.*` on GitHub Packages (when applicable).
- Prefer BCL types and existing Novolis packages over parallel abstractions.
- Document restore and ProjectReference-mode builds without local NuGet folder feeds.

## Non-goals

- Local NuGet folder feeds or committed cross-repo `ProjectReference` into sibling checkouts.
- Avalonia package references outside `Novolis.Avalonia.*`.
- Upward spine dependencies (e.g. Math → Simulation).

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

## Topics

- `dotnet`
- `physics`
- `simulation`
- `novolis`

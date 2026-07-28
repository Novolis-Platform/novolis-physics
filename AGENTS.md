# AGENTS.md

Guidance for AI agents and contributors working in **novolis-physics**.

## What this repo is

Multi-package **.NET 10** force-first physics library published as NuGet packages under the `Novolis.Physics.*` id prefix. Compute forces via `IForceModel`, integrate via `IIntegrator`, orchestrate via `SimulationPipeline`.

**Consumer entry point:** `Novolis.Physics` (meta package). Reference individual packages (`Novolis.Physics.Motion`, `Novolis.Physics.Orbits`, …) when you only need a subset.

## Repository layout

| Path | Role |
|------|------|
| `src/Novolis.Physics/` | Meta package — install this for the full stack |
| `src/Novolis.Physics.Abstractions/` | `IForceModel`, `IIntegrator`, `RigidBodyState`, `IStaticWorld` |
| `src/Novolis.Physics.Motion/` | `SimulationPipeline`, `FixedStepAccumulator` |
| `src/Novolis.Physics.Gravity/` | Point-mass and patched-conic gravity |
| `src/Novolis.Physics.Aerodynamics/` | Atmosphere and lift/drag models |
| `src/Novolis.Physics.Collision.Simple/` | Static mesh BVH, sphere sweeps |
| `src/Novolis.Physics.Ballistics/` | Projectile drag and queries |
| `src/Novolis.Physics.Orbits/` | Two-body orbital helpers (parallel to force pipeline) |
| `src/Novolis.Physics.Astro/` | ly/pc/AU ↔ meters unit bridges |
| `build/` | NuGet packaging metadata |
| `tests/Novolis.Physics.Unit/` | TUnit tests |
| `tests/Novolis.Physics.TestSupport/` | Shared test helpers and fixtures |
| `docs/` | Integration guide, architecture, examples |

## Build and test

```bash
dotnet build Novolis.Physics.slnx -c Release
dotnet run --project tests/Novolis.Physics.Unit -c Release --no-build
pwsh scripts/pack-all.ps1
```

## Git remote (Novolis-Platform)

This repo lives under `d:\novolis\` and **`origin` must be the org repo**:

- `https://github.com/Novolis-Platform/novolis-physics.git`

Before the first `git push` or `gh repo create` in any `novolis-*` repo:

1. Run `git remote -v` and confirm `origin` points at `Novolis-Platform/`.
2. Never create a personal-repo remote for reserved `novolis-*` names.
3. Use `gh repo create --org Novolis-Platform` when bootstrapping a new org repo.

## What not to do

- Commit secrets or local IDE-only config beyond what is already tracked.
- Push implementation work to `frankhaugen/novolis-*` remotes.

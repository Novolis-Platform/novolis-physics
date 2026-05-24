# Versioning policy

Novolis.Physics uses the platform scheme **`SDKYEAR.APIBREAK.FEATURE`** (see [nuget-versioning.md](../../.github/docs/nuget-versioning.md)).

- Version intent: `build/version.json` in this repo (not `eng/`).
- Stable line: `2026.1.0` at cutover.
- Cross-repo dependencies: floating `2026.1.*` in `Directory.Packages.props`.

Public API is the types shipped in NuGet product packages under `src/Novolis.Physics.*` (excluding test-only assemblies).

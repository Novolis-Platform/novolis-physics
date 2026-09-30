using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Per-frame counters from <see cref="SphereInStaticWorldSimulator.Step"/>.</summary>
public struct SphereSimulationStats
{
    /// <summary>Spheres integrated this frame.</summary>
    public int ActiveCount;

    /// <summary>Spheres marked sleeping.</summary>
    public int SleepingCount;

    /// <summary>Sphere–sphere contacts resolved.</summary>
    public int SphereContacts;

    /// <summary>Sphere pairs examined in broad-phase.</summary>
    public int SpherePairChecks;

    /// <summary>Static-world reflection impulses applied.</summary>
    public int IntegratorReflections;

    /// <summary>Internal physics substeps executed.</summary>
    public int PhysicsSubSteps;

    /// <summary>Sphere contact solver iterations used.</summary>
    public int SphereContactIterations;

    /// <summary>Spheres clamped to interior volume.</summary>
    public int ClampedCount;

    /// <summary>True when sphere contact pass was skipped (e.g. all sleeping).</summary>
    public bool SphereContactSkipped;
}

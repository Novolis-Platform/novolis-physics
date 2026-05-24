using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Sphere pile physics with distance joints, optional angular limits, and ragdoll self-collision.</summary>
public sealed class ConstrainedSphereSimulator
{
    private readonly SphereInStaticWorldSimulator _sphereSimulator = new();
    private DistanceJoint[] _joints = [];
    private (int A, int B)[] _skipCollisionPairs = [];

    /// <summary>Underlying sphere-world integration options.</summary>
    public SphereInStaticWorldOptions Options
    {
        get => _sphereSimulator.Options;
        set => _sphereSimulator.Options = value;
    }

    /// <summary>Distance joint solver iterations per constraint pass.</summary>
    public int JointIterations { get; set; } = 12;

    /// <summary>Relaxation iterations after the main joint solve.</summary>
    public int JointRelaxIterations { get; set; } = 6;

    /// <summary>Angular limit solver iterations per constraint pass.</summary>
    public int AngularIterations { get; set; } = 2;

    /// <summary>Ragdoll self-collision iterations per constraint pass.</summary>
    public int InternalCollisionIterations { get; set; } = 5;

    /// <summary>Pre- and post-integration constraint passes.</summary>
    public int ConstraintPasses { get; set; } = 2;

    /// <summary>Statistics from the last integration step.</summary>
    public SphereSimulationStats LastStats { get; private set; }

    /// <summary>Joint corrections applied in the last step.</summary>
    public int LastJointCorrections { get; private set; }

    /// <summary>Angular limit corrections in the last step.</summary>
    public int LastAngularCorrections { get; private set; }

    /// <summary>Internal overlap fixes in the last step.</summary>
    public int LastInternalCollisionFixes { get; private set; }

    /// <summary>Replaces distance joints and rebuilds adjacent skip pairs for self-collision.</summary>
    public void SetJoints(ReadOnlySpan<DistanceJoint> joints)
    {
        _joints = joints.Length == 0 ? [] : joints.ToArray();
        _skipCollisionPairs = RagdollBodyCollision.BuildAdjacentSkipPairs(joints);
    }

    /// <summary>Resets sleeping state on the internal sphere integrator.</summary>
    public void ResetPileState() => _sphereSimulator.ResetPileState();

    /// <summary>Marks the pile as unsettled so contacts reactivate.</summary>
    public void MarkPileUnsettled() => _sphereSimulator.MarkPileUnsettled();

    /// <summary>Depenetrates a slice of spheres after spawn.</summary>
    public void DepenetrateSpawnedRange(
        IList<SphereState> spheres,
        int startIndex,
        int endIndex,
        InteriorClampVolume interior) =>
        _sphereSimulator.DepenetrateSpawnedRange(spheres, startIndex, endIndex, interior);

    /// <summary>Steps spheres with joints only (no angular limits).</summary>
    public void Step(
        BvhStaticWorld staticWorld,
        IList<SphereState> spheres,
        InteriorClampVolume interior,
        float deltaSeconds) =>
        Step(
            staticWorld,
            spheres,
            interior,
            deltaSeconds,
            ReadOnlySpan<SwingLimit>.Empty,
            ReadOnlySpan<HingeLimit>.Empty);

    /// <summary>Steps spheres with distance joints, angular limits, and ragdoll self-collision.</summary>
    public void Step(
        BvhStaticWorld staticWorld,
        IList<SphereState> spheres,
        InteriorClampVolume interior,
        float deltaSeconds,
        ReadOnlySpan<SwingLimit> swingLimits,
        ReadOnlySpan<HingeLimit> hingeLimits)
    {
        var jointCorrections = 0;
        var angularCorrections = 0;
        var collisionFixes = 0;
        var radius = Options.Radius;
        var skipPairs = _skipCollisionPairs;

        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            jointCorrections += SolveJoints(spheres);
            if (swingLimits.Length > 0 || hingeLimits.Length > 0)
                angularCorrections += AngularLimitSolver.Solve(swingLimits, hingeLimits, spheres, AngularIterations);
            collisionFixes += RagdollBodyCollision.ResolveOverlaps(
                spheres,
                radius,
                InternalCollisionIterations,
                separationScale: 1.02f,
                skipPairs);
        }

        _sphereSimulator.Step(staticWorld, spheres, interior, deltaSeconds);

        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            jointCorrections += SolveJoints(spheres);
            if (swingLimits.Length > 0 || hingeLimits.Length > 0)
                angularCorrections += AngularLimitSolver.Solve(swingLimits, hingeLimits, spheres, AngularIterations);
            collisionFixes += RagdollBodyCollision.ResolveOverlaps(
                spheres,
                radius,
                InternalCollisionIterations,
                separationScale: 1.02f,
                skipPairs);
        }

        LastStats = _sphereSimulator.LastStats;
        LastJointCorrections = jointCorrections;
        LastAngularCorrections = angularCorrections;
        LastInternalCollisionFixes = collisionFixes;
    }

    private int SolveJoints(IList<SphereState> spheres)
    {
        if (_joints.Length == 0)
            return 0;

        var corrections = DistanceJointSolver.Solve(_joints, spheres, JointIterations);
        if (JointRelaxIterations > 0)
            corrections += DistanceJointSolver.Solve(_joints, spheres, JointRelaxIterations, positionSlop: 0.002f);
        return corrections;
    }
}

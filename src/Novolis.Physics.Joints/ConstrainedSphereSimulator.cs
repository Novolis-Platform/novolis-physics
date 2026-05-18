using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Sphere pile physics with distance joints and optional ragdoll self-collision.</summary>
public sealed class ConstrainedSphereSimulator
{
    private readonly SphereInStaticWorldSimulator _sphereSimulator = new();
    private DistanceJoint[] _joints = [];

    public SphereInStaticWorldOptions Options
    {
        get => _sphereSimulator.Options;
        set => _sphereSimulator.Options = value;
    }

    public int JointIterations { get; set; } = 12;

    public int JointRelaxIterations { get; set; } = 6;

    public int InternalCollisionIterations { get; set; } = 5;

    public int ConstraintPasses { get; set; } = 2;

    public SphereSimulationStats LastStats { get; private set; }

    public int LastJointCorrections { get; private set; }

    public int LastInternalCollisionFixes { get; private set; }

    public void SetJoints(ReadOnlySpan<DistanceJoint> joints) =>
        _joints = joints.Length == 0 ? [] : joints.ToArray();

    public void ResetPileState() => _sphereSimulator.ResetPileState();

    public void MarkPileUnsettled() => _sphereSimulator.MarkPileUnsettled();

    public void DepenetrateSpawnedRange(
        IList<SphereState> spheres,
        int startIndex,
        int endIndex,
        InteriorClampVolume interior) =>
        _sphereSimulator.DepenetrateSpawnedRange(spheres, startIndex, endIndex, interior);

    public void Step(
        BvhStaticWorld staticWorld,
        IList<SphereState> spheres,
        InteriorClampVolume interior,
        float deltaSeconds)
    {
        var jointCorrections = 0;
        var collisionFixes = 0;
        var radius = Options.Radius;

        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            jointCorrections += SolveJoints(spheres);
            collisionFixes += RagdollBodyCollision.ResolveOverlaps(
                spheres,
                radius,
                InternalCollisionIterations);
        }

        _sphereSimulator.Step(staticWorld, spheres, interior, deltaSeconds);

        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            jointCorrections += SolveJoints(spheres);
            collisionFixes += RagdollBodyCollision.ResolveOverlaps(
                spheres,
                radius,
                InternalCollisionIterations);
        }

        LastStats = _sphereSimulator.LastStats;
        LastJointCorrections = jointCorrections;
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

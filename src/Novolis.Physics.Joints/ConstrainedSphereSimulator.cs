using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Sphere pile physics with optional distance joints solved after each step.</summary>
public sealed class ConstrainedSphereSimulator
{
    private readonly SphereInStaticWorldSimulator _sphereSimulator = new();
    private DistanceJoint[] _joints = [];

    public SphereInStaticWorldOptions Options
    {
        get => _sphereSimulator.Options;
        set => _sphereSimulator.Options = value;
    }

    public int JointIterations { get; set; } = 10;

    public SphereSimulationStats LastStats { get; private set; }

    public int LastJointCorrections { get; private set; }

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
        _sphereSimulator.Step(staticWorld, spheres, interior, deltaSeconds);
        LastStats = _sphereSimulator.LastStats;
        LastJointCorrections = _joints.Length == 0
            ? 0
            : DistanceJointSolver.Solve(_joints, spheres, JointIterations);
    }
}

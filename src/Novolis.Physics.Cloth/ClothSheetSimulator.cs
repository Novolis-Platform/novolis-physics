using System.Numerics;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Cloth;

/// <summary>
/// Cloth step: integrate equal-radius spheres against a static world, then project
/// <see cref="DistanceJoint"/> constraints with pinned anchors reapplied each pass.
/// </summary>
public sealed class ClothSheetSimulator
{
    private readonly SphereInStaticWorldSimulator _sphereSimulator = new();
    private DistanceJoint[] _joints = [];
    private int[] _pinIndices = [];
    private Vector3[] _pinAnchors = [];

    /// <summary>Underlying sphere-world integration options (use radius ≪ spacing).</summary>
    public SphereInStaticWorldOptions Options
    {
        get => _sphereSimulator.Options;
        set => _sphereSimulator.Options = value;
    }

    /// <summary>Distance joint solver iterations per constraint pass.</summary>
    public int JointIterations { get; set; } = 10;

    /// <summary>Relaxation iterations after the main joint solve.</summary>
    public int JointRelaxIterations { get; set; } = 4;

    /// <summary>Constraint projection passes after integration.</summary>
    public int ConstraintPasses { get; set; } = 3;

    /// <summary>
    /// Per-iteration strain correction cap for <see cref="DistanceJointSolver"/>.
    /// Default is large so cloth holds shape under gravity (ragdolls keep 0.35).
    /// </summary>
    public float MaxStrainFraction { get; set; } = 2.5f;

    /// <summary>
    /// Hard maximum edge length as a multiple of rest length after each constraint pass.
    /// Keeps fabric from going doughy when soft projection alone cannot keep up with gravity/wind.
    /// </summary>
    public float MaxStretchRatio { get; set; } = 1.08f;

    /// <summary>Iterations of the hard stretch clamp per constraint pass.</summary>
    public int StretchLimitIterations { get; set; } = 12;

    /// <summary>Optional constant wind acceleration added each step (m/s²).</summary>
    public Vector3 WindAcceleration { get; set; }

    /// <summary>Statistics from the last integration step.</summary>
    public SphereSimulationStats LastStats { get; private set; }

    /// <summary>Joint corrections applied in the last step.</summary>
    public int LastJointCorrections { get; private set; }

    /// <summary>Active distance joints (after cuts).</summary>
    public ReadOnlySpan<DistanceJoint> Joints => _joints;

    /// <summary>Replaces distance joints used by the solver.</summary>
    public void SetJoints(ReadOnlySpan<DistanceJoint> joints) =>
        _joints = joints.Length == 0 ? [] : joints.ToArray();

    /// <summary>Replaces joints from a mutable list (typical after <see cref="ClothCutOps"/>).</summary>
    public void SetJoints(IList<DistanceJoint> joints)
    {
        ArgumentNullException.ThrowIfNull(joints);
        if (joints.Count == 0)
        {
            _joints = [];
            return;
        }

        _joints = new DistanceJoint[joints.Count];
        for (var i = 0; i < joints.Count; i++)
            _joints[i] = joints[i];
    }

    /// <summary>Replaces pinned particle indices and world anchors.</summary>
    public void SetPins(ReadOnlySpan<int> pinIndices, ReadOnlySpan<Vector3> pinAnchors)
    {
        var count = System.Math.Min(pinIndices.Length, pinAnchors.Length);
        _pinIndices = count == 0 ? [] : pinIndices[..count].ToArray();
        _pinAnchors = count == 0 ? [] : pinAnchors[..count].ToArray();
    }

    /// <summary>Resets sleeping state on the internal sphere integrator.</summary>
    public void ResetPileState() => _sphereSimulator.ResetPileState();

    /// <summary>Marks the pile as unsettled so contacts reactivate.</summary>
    public void MarkPileUnsettled() => _sphereSimulator.MarkPileUnsettled();

    /// <summary>Integrates cloth particles for one variable time step.</summary>
    public void Step(
        BvhStaticWorld staticWorld,
        IList<SphereState> spheres,
        InteriorClampVolume interior,
        float deltaSeconds)
    {
        if (deltaSeconds <= 0f || spheres.Count == 0)
        {
            LastStats = default;
            LastJointCorrections = 0;
            return;
        }

        ApplyPins(spheres);
        ApplyWind(spheres, deltaSeconds);
        WakeFreeParticles(spheres);

        // Pre-stabilize before integration so gravity cannot accumulate unbound stretch.
        var corrections = 0;
        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            corrections += SolveJoints(spheres);
            corrections += EnforceStretchLimit(spheres);
            ApplyPins(spheres);
        }

        _sphereSimulator.Step(staticWorld, spheres, interior, deltaSeconds);

        for (var pass = 0; pass < ConstraintPasses; pass++)
        {
            corrections += SolveJoints(spheres);
            corrections += EnforceStretchLimit(spheres);
            ApplyPins(spheres);
        }

        // Final hard clamp so reported lengths respect MaxStretchRatio.
        corrections += EnforceStretchLimit(spheres);
        ApplyPins(spheres);

        LastStats = _sphereSimulator.LastStats;
        LastJointCorrections = corrections;
    }

    private void ApplyPins(IList<SphereState> spheres) =>
        ClothSheetPreset.ApplyPins(spheres, _pinIndices, _pinAnchors);

    private int SolveJoints(IList<SphereState> spheres)
    {
        if (_joints.Length == 0)
            return 0;

        var corrections = DistanceJointSolver.Solve(
            _joints,
            spheres,
            JointIterations,
            maxStrainFraction: MaxStrainFraction);
        if (JointRelaxIterations > 0)
        {
            corrections += DistanceJointSolver.Solve(
                _joints,
                spheres,
                JointRelaxIterations,
                positionSlop: 0.002f,
                maxStrainFraction: MaxStrainFraction);
        }

        return corrections;
    }

    /// <summary>Hard-clamps overstretched edges toward rest length (respects pins).</summary>
    private int EnforceStretchLimit(IList<SphereState> spheres)
    {
        if (_joints.Length == 0 || MaxStretchRatio <= 1f)
            return 0;

        var fixes = 0;
        var iterations = System.Math.Max(1, StretchLimitIterations);
        for (var iter = 0; iter < iterations; iter++)
        {
            var iterFixes = 0;
            foreach (var joint in _joints)
            {
                if ((uint)joint.SphereA >= (uint)spheres.Count || (uint)joint.SphereB >= (uint)spheres.Count)
                    continue;

                var pinA = IsPinned(joint.SphereA);
                var pinB = IsPinned(joint.SphereB);
                if (pinA && pinB)
                    continue;

                var a = spheres[joint.SphereA];
                var b = spheres[joint.SphereB];
                var delta = b.Position - a.Position;
                var distSq = delta.LengthSquared();
                if (distSq < 1e-10f)
                    continue;

                var dist = MathF.Sqrt(distSq);
                var maxLen = joint.RestLength * MaxStretchRatio;
                if (dist <= maxLen)
                    continue;

                var n = delta / dist;
                var excess = dist - maxLen;
                if (pinA)
                    b.Position -= n * excess;
                else if (pinB)
                    a.Position += n * excess;
                else
                {
                    a.Position += n * (excess * 0.5f);
                    b.Position -= n * (excess * 0.5f);
                }

                iterFixes++;
            }

            fixes += iterFixes;
            if (iterFixes == 0)
                break;
        }

        return fixes;
    }

    private void ApplyWind(IList<SphereState> spheres, float deltaSeconds)
    {
        if (WindAcceleration.LengthSquared() < 1e-12f)
            return;

        for (var i = 0; i < spheres.Count; i++)
        {
            if (IsPinned(i))
                continue;

            spheres[i].Velocity += WindAcceleration * deltaSeconds;
        }
    }

    private void WakeFreeParticles(IList<SphereState> spheres)
    {
        for (var i = 0; i < spheres.Count; i++)
        {
            if (IsPinned(i))
                continue;
            spheres[i].IsSleeping = false;
        }

        _sphereSimulator.MarkPileUnsettled();
    }

    private bool IsPinned(int index)
    {
        foreach (var p in _pinIndices)
        {
            if (p == index)
                return true;
        }

        return false;
    }
}

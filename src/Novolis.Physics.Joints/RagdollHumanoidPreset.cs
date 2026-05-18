using System.Numerics;
using System.Runtime.InteropServices;
using Novolis.Physics.Collision.Simple;

namespace Novolis.Physics.Joints;

/// <summary>Standard 11-sphere humanoid ragdoll topology and pose limits.</summary>
public static class RagdollHumanoidPreset
{
    public const int Hip = 0;
    public const int LeftKnee = 1;
    public const int RightKnee = 2;
    public const int Chest = 3;
    public const int Head = 4;
    public const int LeftShoulder = 5;
    public const int RightShoulder = 6;
    public const int LeftHand = 7;
    public const int RightHand = 8;
    public const int LeftFoot = 9;
    public const int RightFoot = 10;
    public const int SphereCount = 11;

    private const float Deg = MathF.PI / 180f;

    public static void BuildStanding(
        Vector3 groundPoint,
        IList<SphereState> spheres,
        IList<DistanceJoint> joints,
        IList<SwingLimit> swingLimits,
        IList<HingeLimit> hingeLimits,
        float runtimeStiffness = 0.65f)
    {
        spheres.Clear();
        joints.Clear();
        swingLimits.Clear();
        hingeLimits.Clear();

        var hip = groundPoint + new Vector3(0f, 1.02f, 0f);
        var chest = hip + new Vector3(0f, 0.5f, 0.02f);
        var head = chest + new Vector3(0f, 0.4f, 0f);
        var lKnee = hip + new Vector3(-0.2f, -0.5f, 0.06f);
        var rKnee = hip + new Vector3(0.2f, -0.5f, 0.06f);
        var lFoot = lKnee + new Vector3(0f, -0.42f, 0.1f);
        var rFoot = rKnee + new Vector3(0f, -0.42f, 0.1f);
        var lShoulder = chest + new Vector3(-0.3f, 0.1f, 0.1f);
        var rShoulder = chest + new Vector3(0.3f, 0.1f, 0.1f);
        var lHand = lShoulder + new Vector3(-0.26f, -0.06f, 0.16f);
        var rHand = rShoulder + new Vector3(0.26f, -0.06f, 0.16f);

        AddSphere(spheres, hip);
        AddSphere(spheres, lKnee);
        AddSphere(spheres, rKnee);
        AddSphere(spheres, chest);
        AddSphere(spheres, head);
        AddSphere(spheres, lShoulder);
        AddSphere(spheres, rShoulder);
        AddSphere(spheres, lHand);
        AddSphere(spheres, rHand);
        AddSphere(spheres, lFoot);
        AddSphere(spheres, rFoot);

        Link(joints, spheres, Hip, Chest);
        Link(joints, spheres, Chest, Head);
        Link(joints, spheres, Hip, LeftKnee);
        Link(joints, spheres, Hip, RightKnee);
        Link(joints, spheres, LeftKnee, LeftFoot);
        Link(joints, spheres, RightKnee, RightFoot);
        Link(joints, spheres, Chest, LeftShoulder);
        Link(joints, spheres, Chest, RightShoulder);
        Link(joints, spheres, LeftShoulder, LeftHand);
        Link(joints, spheres, RightShoulder, RightHand);

        BuildLimits(spheres, swingLimits, hingeLimits, runtimeStiffness);
    }

    public static void BuildLimits(
        IList<SphereState> spheres,
        IList<SwingLimit> swingLimits,
        IList<HingeLimit> hingeLimits,
        float stiffness)
    {
        swingLimits.Clear();
        hingeLimits.Clear();

        AddLocalSwing(spheres, swingLimits, Hip, Chest, frameRef: Chest, maxDegrees: 32f, stiffness);
        AddLocalSwing(spheres, swingLimits, Chest, Head, frameRef: Hip, maxDegrees: 45f, stiffness);
        AddLocalSwing(spheres, swingLimits, Chest, LeftShoulder, frameRef: Hip, maxDegrees: 70f, stiffness);
        AddLocalSwing(spheres, swingLimits, Chest, RightShoulder, frameRef: Hip, maxDegrees: 70f, stiffness);

        AddLocalHinge(spheres, hingeLimits, Hip, LeftKnee, frameRef: Chest, lateralSign: 1f, minDeg: -8f, maxDeg: 105f, stiffness);
        AddLocalHinge(spheres, hingeLimits, Hip, RightKnee, frameRef: Chest, lateralSign: -1f, minDeg: -8f, maxDeg: 105f, stiffness);

        AddLocalHinge(spheres, hingeLimits, LeftKnee, LeftFoot, frameRef: Hip, lateralSign: 1f, minDeg: -5f, maxDeg: 95f, stiffness);
        AddLocalHinge(spheres, hingeLimits, RightKnee, RightFoot, frameRef: Hip, lateralSign: -1f, minDeg: -5f, maxDeg: 95f, stiffness);

        AddLocalElbow(spheres, hingeLimits, LeftShoulder, LeftHand, frameRef: Chest, lateralSign: 1f, stiffness);
        AddLocalElbow(spheres, hingeLimits, RightShoulder, RightHand, frameRef: Chest, lateralSign: -1f, stiffness);
    }

    public static void StabilizeSpawn(
        IList<SphereState> spheres,
        ReadOnlySpan<DistanceJoint> joints,
        InteriorClampVolume clamp,
        ConstrainedSphereSimulator simulator,
        float spawnStiffness = 0.85f)
    {
        var spawnSwings = new List<SwingLimit>();
        var spawnHinges = new List<HingeLimit>();
        BuildLimits(spheres, spawnSwings, spawnHinges, spawnStiffness);

        for (var i = 0; i < 32; i++)
        {
            DistanceJointSolver.Solve(joints, spheres, 10);
            AngularLimitSolver.Solve(
                CollectionsMarshal.AsSpan(spawnSwings),
                CollectionsMarshal.AsSpan(spawnHinges),
                spheres,
                2);
        }

        simulator.DepenetrateSpawnedRange(spheres, 0, spheres.Count, clamp);

        foreach (var sphere in spheres)
        {
            sphere.Velocity = Vector3.Zero;
            sphere.IsSleeping = false;
        }

        simulator.ResetPileState();
    }

    private static void AddSphere(IList<SphereState> spheres, Vector3 position) =>
        spheres.Add(new SphereState(position, Vector3.Zero));

    private static void Link(IList<DistanceJoint> joints, IList<SphereState> spheres, int a, int b) =>
        joints.Add(new DistanceJoint(a, b, Vector3.Distance(spheres[a].Position, spheres[b].Position), 1f));

    private static void AddLocalSwing(
        IList<SphereState> spheres,
        IList<SwingLimit> limits,
        int parent,
        int child,
        int frameRef,
        float maxDegrees,
        float stiffness)
    {
        if (!BoneFrame.TryCreate(spheres[parent].Position, spheres[frameRef].Position, out var frame))
            return;

        var restLocal = frame.WorldToLocal(BoneDirection(spheres, parent, child));
        limits.Add(SwingLimit.CreateLocal(parent, child, frameRef, restLocal, maxDegrees * Deg, stiffness));
    }

    private static void AddLocalHinge(
        IList<SphereState> spheres,
        IList<HingeLimit> limits,
        int parent,
        int child,
        int frameRef,
        float lateralSign,
        float minDeg,
        float maxDeg,
        float stiffness)
    {
        if (!BoneFrame.TryCreate(spheres[parent].Position, spheres[frameRef].Position, out var frame))
            return;

        var restLocal = frame.WorldToLocal(BoneDirection(spheres, parent, child));
        var axisLocal = Vector3.Normalize(new Vector3(lateralSign, 0f, 0f));
        limits.Add(HingeLimit.CreateLocal(
            parent,
            child,
            frameRef,
            axisLocal,
            restLocal,
            minDeg * Deg,
            maxDeg * Deg,
            stiffness));
    }

    private static void AddLocalElbow(
        IList<SphereState> spheres,
        IList<HingeLimit> limits,
        int parent,
        int child,
        int frameRef,
        float lateralSign,
        float stiffness)
    {
        if (!BoneFrame.TryCreate(spheres[parent].Position, spheres[frameRef].Position, out var frame))
            return;

        var restLocal = frame.WorldToLocal(BoneDirection(spheres, parent, child));
        var restWorld = BoneDirection(spheres, parent, child);
        var axisWorld = Vector3.Normalize(Vector3.Cross(restWorld, Vector3.UnitZ) + new Vector3(0.02f * lateralSign, 0f, 0f));
        if (float.IsNaN(axisWorld.X))
            axisWorld = new Vector3(lateralSign, 0f, 0f);
        var axisLocal = frame.WorldToLocal(axisWorld);
        limits.Add(HingeLimit.CreateLocal(
            parent,
            child,
            frameRef,
            axisLocal,
            restLocal,
            minRadians: -6f * Deg,
            maxRadians: 132f * Deg,
            stiffness));
    }

    private static Vector3 BoneDirection(IList<SphereState> spheres, int parent, int child) =>
        Vector3.Normalize(spheres[child].Position - spheres[parent].Position);
}

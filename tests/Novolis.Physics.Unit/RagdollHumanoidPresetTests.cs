using System.Numerics;
using System.Runtime.InteropServices;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Unit;

public sealed class RagdollHumanoidPresetTests
{
    [Test]
    public async Task BuildStanding_CreatesExpectedTopology()
    {
        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var swings = new List<SwingLimit>();
        var hinges = new List<HingeLimit>();

        RagdollHumanoidPreset.BuildStanding(Vector3.Zero, spheres, joints, swings, hinges);

        await Assert.That(spheres.Count).IsEqualTo(RagdollHumanoidPreset.SphereCount);
        await Assert.That(joints.Count).IsEqualTo(10);
        await Assert.That(swings.Count).IsGreaterThan(0);
        await Assert.That(hinges.Count).IsGreaterThanOrEqualTo(6);
    }

    [Test]
    public async Task BuildLimits_PopulatesExpectedCounts()
    {
        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var swings = new List<SwingLimit>();
        var hinges = new List<HingeLimit>();
        RagdollHumanoidPreset.BuildStanding(Vector3.Zero, spheres, joints, swings, hinges);

        swings.Clear();
        hinges.Clear();
        RagdollHumanoidPreset.BuildLimits(spheres, swings, hinges, stiffness: 0.5f);

        await Assert.That(swings.Count).IsEqualTo(4);
        await Assert.That(hinges.Count).IsEqualTo(6);
    }

    [Test]
    public async Task StabilizeSpawn_ZerosVelocitiesAndDepenetrates()
    {
        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var swings = new List<SwingLimit>();
        var hinges = new List<HingeLimit>();
        RagdollHumanoidPreset.BuildStanding(new Vector3(0f, 0f, 0f), spheres, joints, swings, hinges);
        foreach (var s in spheres)
            s.Velocity = new Vector3(1f, 2f, 3f);

        var sim = new ConstrainedSphereSimulator { Options = { Radius = 0.12f, MaxSpeedMps = 10f } };
        var clamp = new InteriorClampVolume { MinX = -2, MaxX = 2, MinY = 0, MaxY = 3, MinZ = -2, MaxZ = 2 };
        RagdollHumanoidPreset.StabilizeSpawn(spheres, CollectionsMarshal.AsSpan(joints), clamp, sim);

        await Assert.That(spheres.All(s => s.Velocity == Vector3.Zero)).IsTrue();
        await Assert.That(spheres[RagdollHumanoidPreset.Hip].Position.Y).IsGreaterThan(0.5f);
    }
}

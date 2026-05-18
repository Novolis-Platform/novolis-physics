using System.Numerics;
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
}

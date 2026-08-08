using System.Numerics;
using System.Runtime.InteropServices;
using Novolis.Math.Geometry;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Cloth;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Unit;

public sealed class ClothCutOpsTests
{
    [Test]
    public async Task CutWithBlade_Midline_SplitsStructuralLinksAcrossCenter()
    {
        var options = new ClothSheetOptions
        {
            Columns = 6,
            Rows = 4,
            Spacing = 0.2f,
            IncludeShear = false,
            IncludeBend = false,
            PinMode = ClothPinMode.None,
        };

        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var pins = new List<int>();
        var anchors = new List<Vector3>();

        ClothSheetPreset.BuildHanging(
            new Vector3(0f, 2f, 0f),
            Vector3.UnitX,
            -Vector3.UnitY,
            options,
            spheres,
            joints,
            pins,
            anchors);

        var before = joints.Count;
        // Vertical blade along the sheet height at x = 0.5 (between col 2 and 3).
        var blade = new ClothBlade(
            heel: new Vector3(0.5f, 2.1f, 0f),
            tip: new Vector3(0.5f, 1.1f, 0f),
            halfThickness: 0.06f);

        var result = ClothCutOps.CutWithBlade(joints, spheres, blade);

        await Assert.That(result.SeveredJointCount).IsGreaterThan(0);
        await Assert.That(joints.Count).IsEqualTo(before - result.SeveredJointCount);

        // No remaining structural link should cross the cut plane x=0.5 between adjacent columns.
        foreach (var joint in joints)
        {
            var a = spheres[joint.SphereA].Position.X;
            var b = spheres[joint.SphereB].Position.X;
            var crosses = (a < 0.5f && b > 0.5f) || (b < 0.5f && a > 0.5f);
            await Assert.That(crosses).IsFalse();
        }
    }

    [Test]
    public async Task CutWithBlast_AndImpulse_SeversClusterAndFlingsParticles()
    {
        var options = new ClothSheetOptions
        {
            Columns = 8,
            Rows = 6,
            Spacing = 0.15f,
            IncludeShear = true,
            IncludeBend = false,
            PinMode = ClothPinMode.None,
        };

        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        ClothSheetPreset.BuildHanging(
            Vector3.Zero,
            Vector3.UnitX,
            -Vector3.UnitY,
            options,
            spheres,
            joints,
            new List<int>(),
            new List<Vector3>());

        var before = joints.Count;
        var center = spheres[ClothSheetPreset.Index(3, 2, 8)].Position;
        var blast = new ClothBlast(center, radius: 0.28f, impulseSpeed: 4f);

        var cut = ClothCutOps.CutWithBlast(joints, spheres, blast);
        var kicked = ClothCutOps.ApplyBlastImpulse(spheres, blast);

        await Assert.That(cut.SeveredJointCount).IsGreaterThan(3);
        await Assert.That(joints.Count).IsLessThan(before);
        await Assert.That(kicked).IsGreaterThan(0);

        var speed = spheres[ClothSheetPreset.Index(3, 2, 8)].Velocity.Length();
        await Assert.That(speed).IsGreaterThan(0.5f);
    }

    [Test]
    public async Task ClothSheetSimulator_EnforcesMaxStretchRatio_UnderGravity()
    {
        var options = new ClothSheetOptions
        {
            Columns = 6,
            Rows = 5,
            Spacing = 0.15f,
            PinMode = ClothPinMode.TopRow,
            IncludeShear = true,
            IncludeBend = true,
        };

        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var pins = new List<int>();
        var anchors = new List<Vector3>();
        ClothSheetPreset.BuildHanging(
            new Vector3(0f, 3f, 0f),
            Vector3.UnitX,
            -Vector3.UnitY,
            options,
            spheres,
            joints,
            pins,
            anchors);

        var sim = new ClothSheetSimulator
        {
            Options =
            {
                Radius = 0.03f,
                LinearDragPerSecond = 0.5,
                SleepSpeedThreshold = 0f,
                MaxSpeedMps = 20f,
            },
            JointIterations = 20,
            ConstraintPasses = 4,
            MaxStrainFraction = 3f,
            MaxStretchRatio = 1.08f,
            StretchLimitIterations = 24,
            WindAcceleration = new Vector3(4f, 0f, 0f),
        };
        sim.SetJoints(joints);
        sim.SetPins(CollectionsMarshal.AsSpan(pins), CollectionsMarshal.AsSpan(anchors));

        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var clamp = new InteriorClampVolume
        {
            MinX = -8f, MaxX = 8f, MinY = -1f, MaxY = 10f, MinZ = -8f, MaxZ = 8f,
        };

        for (var i = 0; i < 120; i++)
            sim.Step(world, spheres, clamp, 1f / 60f);

        var maxStretch = 0f;
        foreach (var joint in joints)
        {
            // Simulator holds its own joint copy — measure live sphere distances vs rest from original list.
            var d = Vector3.Distance(spheres[joint.SphereA].Position, spheres[joint.SphereB].Position);
            maxStretch = MathF.Max(maxStretch, d / joint.RestLength);
        }

        await Assert.That(maxStretch).IsLessThanOrEqualTo(1.12f);
    }
}

using System.Numerics;
using System.Runtime.InteropServices;
using Novolis.Math.Geometry;
using Novolis.Physics.Collision.Simple;
using Novolis.Physics.Cloth;
using Novolis.Physics.Joints;

namespace Novolis.Physics.Unit;

public sealed class ClothSheetPresetTests
{
    [Test]
    public async Task BuildHanging_CreatesGridWithStructuralShearBendAndTopPins()
    {
        var options = new ClothSheetOptions
        {
            Columns = 4,
            Rows = 3,
            Spacing = 0.2f,
            IncludeShear = true,
            IncludeBend = true,
            PinMode = ClothPinMode.TopRow,
        };

        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var pins = new List<int>();
        var anchors = new List<Vector3>();

        ClothSheetPreset.BuildHanging(
            origin: new Vector3(0f, 2f, 0f),
            right: Vector3.UnitX,
            down: -Vector3.UnitY,
            options,
            spheres,
            joints,
            pins,
            anchors);

        await Assert.That(spheres.Count).IsEqualTo(12);
        await Assert.That(pins.Count).IsEqualTo(4);
        await Assert.That(anchors.Count).IsEqualTo(4);

        // Structural: horizontal 3*3 + vertical 4*2 = 17
        // Shear: 3*2*2 = 12
        // Bend: horizontal skip 2*3 + vertical skip 1*4 = 10
        await Assert.That(joints.Count).IsEqualTo(17 + 12 + 10);

        await Assert.That(spheres[0].Position).IsEqualTo(new Vector3(0f, 2f, 0f));
        await Assert.That(spheres[ClothSheetPreset.Index(3, 2, 4)].Position.Y).IsEqualTo(1.6f).Within(1e-4f);
    }

    [Test]
    public async Task ApplyPins_RestoresAnchors()
    {
        var spheres = new List<SphereState>
        {
            new(new Vector3(1f, 1f, 1f), new Vector3(2f, 0f, 0f)),
            new(new Vector3(0f, 0f, 0f), Vector3.Zero),
        };
        int[] pins = [0];
        Vector3[] anchors = [new Vector3(5f, 5f, 5f)];

        ClothSheetPreset.ApplyPins(spheres, pins, anchors);

        await Assert.That(spheres[0].Position).IsEqualTo(new Vector3(5f, 5f, 5f));
        await Assert.That(spheres[0].Velocity).IsEqualTo(Vector3.Zero);
    }

    [Test]
    public async Task WriteTriangleIndices_BuildsTwoTrisPerCell()
    {
        var indices = ClothSheetPreset.CreateTriangleIndices(3, 2);
        await Assert.That(indices.Length).IsEqualTo(12);
        await Assert.That(indices[0]).IsEqualTo(0);
        await Assert.That(indices[1]).IsEqualTo(3);
        await Assert.That(indices[2]).IsEqualTo(1);
    }

    [Test]
    public async Task ClothSheetSimulator_PinnedTopRow_HangsWithoutFallingPins()
    {
        var options = new ClothSheetOptions
        {
            Columns = 5,
            Rows = 4,
            Spacing = 0.15f,
            PinMode = ClothPinMode.TopRow,
            IncludeBend = false,
        };

        var spheres = new List<SphereState>();
        var joints = new List<DistanceJoint>();
        var pins = new List<int>();
        var anchors = new List<Vector3>();
        var origin = new Vector3(0f, 3f, 0f);

        ClothSheetPreset.BuildHanging(
            origin,
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
                LinearDragPerSecond = 0.4,
                SphereRestitution = 0f,
                StaticRestitution = 0f,
                SleepSpeedThreshold = 0f,
                MaxSpeedMps = 20f,
            },
            JointIterations = 8,
            JointRelaxIterations = 2,
            ConstraintPasses = 2,
            WindAcceleration = new Vector3(1.5f, 0f, 0.4f),
        };
        sim.SetJoints(CollectionsMarshal.AsSpan(joints));
        sim.SetPins(CollectionsMarshal.AsSpan(pins), CollectionsMarshal.AsSpan(anchors));

        var world = new BvhStaticWorld(new TriangleMesh([], []));
        var clamp = new InteriorClampVolume
        {
            MinX = -10f,
            MaxX = 10f,
            MinY = -1f,
            MaxY = 10f,
            MinZ = -10f,
            MaxZ = 10f,
        };

        for (var i = 0; i < 90; i++)
            sim.Step(world, spheres, clamp, 1f / 60f);

        for (var i = 0; i < pins.Count; i++)
        {
            await Assert.That(Vector3.Distance(spheres[pins[i]].Position, anchors[i])).IsLessThan(1e-4f);
        }

        var free = spheres[ClothSheetPreset.Index(2, 3, 5)];
        await Assert.That(free.Position.Y).IsLessThan(origin.Y - 0.2f);
        await Assert.That(sim.LastJointCorrections).IsGreaterThan(0);
    }
}

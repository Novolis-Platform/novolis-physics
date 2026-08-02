using Novolis.Physics.TestSupport;
using Novolis.Physics.Collision.Simple;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Unit;

/// <summary>Static triangle meshes for collision tests (axis-aligned rooms, table + bumpers).</summary>
internal static class CollisionTestGeometry
{
    /// <summary>Closed axis-aligned room: interior is <c>min &lt; position &lt; max</c> on each axis (strict inequality for open set; walls sit on the boundary planes).</summary>
    internal static BvhStaticWorld BuildAxisAlignedRoom(Vector3 min, Vector3 max, double edgePad = 24.0)
    {
        var verts = new List<Vector3>();
        var tris = new List<int>();

        var mx0 = min.X;
        var my0 = min.Y;
        var mz0 = min.Z;
        var mx1 = max.X;
        var my1 = max.Y;
        var mz1 = max.Z;

        var ex0 = mx0 - edgePad;
        var ey0 = my0 - edgePad;
        var ez0 = mz0 - edgePad;
        var ex1 = mx1 + edgePad;
        var ey1 = my1 + edgePad;
        var ez1 = mz1 + edgePad;

        // x = mx0 (left): outward from interior is -X; CCW from +X view uses (p00,p01,p11,p10) style
        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(mx0, ey0, ez0),
            PhysicsTestVectors.V(mx0, ey0, ez1),
            PhysicsTestVectors.V(mx0, ey1, ez1),
            PhysicsTestVectors.V(mx0, ey1, ez0));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(mx1, ey0, ez0),
            PhysicsTestVectors.V(mx1, ey1, ez0),
            PhysicsTestVectors.V(mx1, ey1, ez1),
            PhysicsTestVectors.V(mx1, ey0, ez1));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(ex0, my0, ez0),
            PhysicsTestVectors.V(ex1, my0, ez0),
            PhysicsTestVectors.V(ex1, my0, ez1),
            PhysicsTestVectors.V(ex0, my0, ez1));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(ex0, my1, ez0),
            PhysicsTestVectors.V(ex0, my1, ez1),
            PhysicsTestVectors.V(ex1, my1, ez1),
            PhysicsTestVectors.V(ex1, my1, ez0));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(ex0, ey0, mz0),
            PhysicsTestVectors.V(ex0, ey1, mz0),
            PhysicsTestVectors.V(ex1, ey1, mz0),
            PhysicsTestVectors.V(ex1, ey0, mz0));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(ex0, ey0, mz1),
            PhysicsTestVectors.V(ex1, ey0, mz1),
            PhysicsTestVectors.V(ex1, ey1, mz1),
            PhysicsTestVectors.V(ex0, ey1, mz1));

        return new BvhStaticWorld(new TriangleMesh(verts.ToArray(), tris.ToArray()));
    }

    /// <summary>Billiards-style box: floor at y=0, four vertical bumpers on a rectangle in XZ, optional ceiling (closed room for rally tests).</summary>
    internal static BvhStaticWorld BuildBilliardsTableWithBumpers(
        double tableX0,
        double tableX1,
        double tableZ0,
        double tableZ1,
        double bumperHeight,
        double padBeyondTable,
        double? ceilingY = null)
    {
        var verts = new List<Vector3>();
        var tris = new List<int>();

        var fx0 = tableX0 - padBeyondTable;
        var fx1 = tableX1 + padBeyondTable;
        var fz0 = tableZ0 - padBeyondTable;
        var fz1 = tableZ1 + padBeyondTable;

        // Floor y=0, normal +Y
        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(fx0, 0, fz0),
            PhysicsTestVectors.V(fx1, 0, fz0),
            PhysicsTestVectors.V(fx1, 0, fz1),
            PhysicsTestVectors.V(fx0, 0, fz1));

        // Bumper x = tableX0 (thin wall in +X interior)
        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(tableX0, 0, fz0),
            PhysicsTestVectors.V(tableX0, bumperHeight, fz0),
            PhysicsTestVectors.V(tableX0, bumperHeight, fz1),
            PhysicsTestVectors.V(tableX0, 0, fz1));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(tableX1, 0, fz0),
            PhysicsTestVectors.V(tableX1, 0, fz1),
            PhysicsTestVectors.V(tableX1, bumperHeight, fz1),
            PhysicsTestVectors.V(tableX1, bumperHeight, fz0));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(fx0, 0, tableZ0),
            PhysicsTestVectors.V(fx1, 0, tableZ0),
            PhysicsTestVectors.V(fx1, bumperHeight, tableZ0),
            PhysicsTestVectors.V(fx0, bumperHeight, tableZ0));

        AddQuad(
            verts,
            tris,
            PhysicsTestVectors.V(fx0, 0, tableZ1),
            PhysicsTestVectors.V(fx0, bumperHeight, tableZ1),
            PhysicsTestVectors.V(fx1, bumperHeight, tableZ1),
            PhysicsTestVectors.V(fx1, 0, tableZ1));

        if (ceilingY is double cy && cy > 0.01)
        {
            AddQuad(
                verts,
                tris,
                PhysicsTestVectors.V(fx0, cy, fz0),
                PhysicsTestVectors.V(fx1, cy, fz0),
                PhysicsTestVectors.V(fx1, cy, fz1),
                PhysicsTestVectors.V(fx0, cy, fz1));

            // Close the padded footprint so elastic rallies cannot drift off the felt into infinity.
            AddQuad(
                verts,
                tris,
                PhysicsTestVectors.V(fx0, 0, fz0),
                PhysicsTestVectors.V(fx0, 0, fz1),
                PhysicsTestVectors.V(fx0, cy, fz1),
                PhysicsTestVectors.V(fx0, cy, fz0));
            AddQuad(
                verts,
                tris,
                PhysicsTestVectors.V(fx1, 0, fz0),
                PhysicsTestVectors.V(fx1, cy, fz0),
                PhysicsTestVectors.V(fx1, cy, fz1),
                PhysicsTestVectors.V(fx1, 0, fz1));
            AddQuad(
                verts,
                tris,
                PhysicsTestVectors.V(fx0, 0, fz0),
                PhysicsTestVectors.V(fx0, cy, fz0),
                PhysicsTestVectors.V(fx1, cy, fz0),
                PhysicsTestVectors.V(fx1, 0, fz0));
            AddQuad(
                verts,
                tris,
                PhysicsTestVectors.V(fx0, 0, fz1),
                PhysicsTestVectors.V(fx1, 0, fz1),
                PhysicsTestVectors.V(fx1, cy, fz1),
                PhysicsTestVectors.V(fx0, cy, fz1));
        }

        return new BvhStaticWorld(new TriangleMesh(verts.ToArray(), tris.ToArray()));
    }

    private static void AddQuad(List<Vector3> verts, List<int> tris, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        var o = verts.Count;
        verts.Add(a);
        verts.Add(b);
        verts.Add(c);
        verts.Add(d);
        tris.Add(o);
        tris.Add(o + 1);
        tris.Add(o + 2);
        tris.Add(o);
        tris.Add(o + 2);
        tris.Add(o + 3);
    }
}

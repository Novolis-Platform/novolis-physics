using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Indexed triangle soup; immutable after construction.</summary>
public sealed class StaticTriangleMesh
{
    public StaticTriangleMesh(Vector3[] vertices, int[] triangleIndices)
    {
        if (triangleIndices.Length % 3 != 0)
        {
            throw new ArgumentException("Triangle index count must be a multiple of 3.", nameof(triangleIndices));
        }

        Vertices = vertices;
        TriangleIndices = triangleIndices;
        TriangleCount = triangleIndices.Length / 3;
    }

    public Vector3[] Vertices { get; }
    public int[] TriangleIndices { get; }
    public int TriangleCount { get; }

    public void GetTriangle(int triangleIndex, out Vector3 v0, out Vector3 v1, out Vector3 v2)
    {
        var i = triangleIndex * 3;
        v0 = Vertices[TriangleIndices[i]];
        v1 = Vertices[TriangleIndices[i + 1]];
        v2 = Vertices[TriangleIndices[i + 2]];
    }

    public AxisAlignedBox3 TriangleBounds(int triangleIndex)
    {
        GetTriangle(triangleIndex, out var v0, out var v1, out var v2);
        var box = AxisAlignedBox3.FromMinMax(v0, v0);
        box = AxisAlignedBox3.Expand(box, v1);
        box = AxisAlignedBox3.Expand(box, v2);
        return box;
    }

    public AxisAlignedBox3 MeshBounds()
    {
        if (Vertices.Length == 0)
        {
            return new AxisAlignedBox3(Vector3.Zero, Vector3.Zero);
        }

        var b = AxisAlignedBox3.FromMinMax(Vertices[0], Vertices[0]);
        foreach (var v in Vertices)
        {
            b = AxisAlignedBox3.Expand(b, v);
        }

        return b;
    }
}

using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Indexed triangle soup; immutable after construction.</summary>
public sealed class StaticTriangleMesh
{
    /// <summary>Initializes mesh geometry from vertices and triangle indices.</summary>
    public StaticTriangleMesh(Vector3[] vertices, int[] triangleIndices)
    {
        if (triangleIndices.Length % 3 != 0)
            throw new ArgumentException("Triangle index count must be a multiple of 3.", nameof(triangleIndices));

        Vertices = vertices;
        TriangleIndices = triangleIndices;
        TriangleCount = triangleIndices.Length / 3;
    }

    /// <summary>Vertex positions in world space.</summary>
    public Vector3[] Vertices { get; }

    /// <summary>Triangle corner indices (three per triangle).</summary>
    public int[] TriangleIndices { get; }

    /// <summary>Number of triangles in the mesh.</summary>
    public int TriangleCount { get; }

    /// <summary>Returns the three vertices of triangle <paramref name="triangleIndex"/>.</summary>
    public void GetTriangle(int triangleIndex, out Vector3 v0, out Vector3 v1, out Vector3 v2)
    {
        var i = triangleIndex * 3;
        v0 = Vertices[TriangleIndices[i]];
        v1 = Vertices[TriangleIndices[i + 1]];
        v2 = Vertices[TriangleIndices[i + 2]];
    }

    /// <summary>Axis-aligned bounds of triangle <paramref name="triangleIndex"/>.</summary>
    public AxisAlignedBox3 TriangleBounds(int triangleIndex)
    {
        GetTriangle(triangleIndex, out var v0, out var v1, out var v2);
        var box = AxisAlignedBox3.FromMinMax(v0, v0);
        box = AxisAlignedBox3.Expand(box, v1);
        box = AxisAlignedBox3.Expand(box, v2);
        return box;
    }

    /// <summary>Axis-aligned bounds enclosing all vertices.</summary>
    public AxisAlignedBox3 MeshBounds()
    {
        if (Vertices.Length == 0)
            return new AxisAlignedBox3(Vector3.Zero, Vector3.Zero);

        var b = AxisAlignedBox3.FromMinMax(Vertices[0], Vertices[0]);
        foreach (var v in Vertices)
            b = AxisAlignedBox3.Expand(b, v);

        return b;
    }
}

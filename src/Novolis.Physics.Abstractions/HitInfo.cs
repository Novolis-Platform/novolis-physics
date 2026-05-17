using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>Ray or sweep hit: parametric distance, contact point, outward normal, and primitive index.</summary>
public readonly struct HitInfo
{
    public HitInfo(double distance, Vector3 point, Vector3 normal, int primitiveIndex)
    {
        Distance = distance;
        Point = point;
        Normal = Vector3.Normalize(normal);
        PrimitiveIndex = primitiveIndex;
    }

    public double Distance { get; }
    public Vector3 Point { get; }
    public Vector3 Normal { get; }
    public int PrimitiveIndex { get; }
}

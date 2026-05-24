using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Abstractions;

/// <summary>Ray or sweep hit: parametric distance, contact point, outward normal, and primitive index.</summary>
public readonly struct HitInfo
{
    /// <summary>Initializes a hit result.</summary>
    /// <param name="distance">Parametric distance along the ray or sweep (meters).</param>
    /// <param name="point">Contact point in world space (meters).</param>
    /// <param name="normal">Outward-facing unit normal at the contact.</param>
    /// <param name="primitiveIndex">Index of the struck primitive in the static world.</param>
    public HitInfo(double distance, Vector3 point, Vector3 normal, int primitiveIndex)
    {
        Distance = distance;
        Point = point;
        Normal = Vector3.Normalize(normal);
        PrimitiveIndex = primitiveIndex;
    }

    /// <summary>Parametric distance along the ray or sweep (meters).</summary>
    public double Distance { get; }

    /// <summary>Contact point in world space (meters).</summary>
    public Vector3 Point { get; }

    /// <summary>Outward-facing unit normal at the contact.</summary>
    public Vector3 Normal { get; }

    /// <summary>Index of the struck primitive in the static world.</summary>
    public int PrimitiveIndex { get; }
}

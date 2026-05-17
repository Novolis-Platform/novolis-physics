namespace Novolis.Physics.Collision.Simple;

/// <summary>Axis-aligned volume used to clamp sphere centers inside a room.</summary>
public readonly struct InteriorClampVolume
{
    public float MinX { get; init; }
    public float MaxX { get; init; }
    public float MinY { get; init; }
    public float MaxY { get; init; }
    public float MinZ { get; init; }
    public float MaxZ { get; init; }
}

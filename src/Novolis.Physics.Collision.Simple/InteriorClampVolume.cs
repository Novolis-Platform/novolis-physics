namespace Novolis.Physics.Collision.Simple;

/// <summary>Axis-aligned volume used to clamp sphere centers inside a room.</summary>
public readonly struct InteriorClampVolume
{
    /// <summary>MinX.</summary>
    public float MinX { get; init; }
    /// <summary>MinY.</summary>
    public float MaxX { get; init; }
    /// <summary>MinZ.</summary>
    public float MinY { get; init; }
    /// <summary>MaxY.</summary>
    public float MaxY { get; init; }
    /// <summary>MaxZ.</summary>
    public float MinZ { get; init; }
    /// <summary>MaxZ.</summary>
    public float MaxZ { get; init; }
}

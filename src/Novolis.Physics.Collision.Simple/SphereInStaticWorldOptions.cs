using System.Numerics;

namespace Novolis.Physics.Collision.Simple;

/// <summary>Configuration for <see cref="SphereInStaticWorldSimulator"/>.</summary>
public sealed class SphereInStaticWorldOptions
{
    /// <summary>Sphere radius (meters).</summary>
    public float Radius { get; set; } = 0.22f;

    /// <summary>World gravity (m/s²).</summary>
    public Vector3 Gravity { get; set; } = new(0f, -9.80665f, 0f);

    /// <summary>Linear drag coefficient (1/s).</summary>
    public double LinearDragPerSecond { get; set; } = 0.048;

    /// <summary>Restitution for static mesh contacts.</summary>
    public double StaticRestitution { get; set; } = 0.82;

    /// <summary>Restitution for sphere–sphere contacts.</summary>
    public float SphereRestitution { get; set; } = 0.88f;

    /// <summary>Ground friction decay rate (1/s).</summary>
    public double GroundFrictionPerSecond { get; set; } = 9.5;

    /// <summary>Floor height for ground plane contact (meters).</summary>
    public float FloorHeight { get; set; }

    /// <summary>Slack before ground contact activates (meters).</summary>
    public float GroundContactSlack { get; set; } = 0.05f;

    /// <summary>Speed below which spheres may sleep (m/s).</summary>
    public float SleepSpeedThreshold { get; set; } = 0.12f;

    /// <summary>Maximum linear speed clamp (m/s).</summary>
    public float MaxSpeedMps { get; set; } = 18f;

    /// <summary>Uniform-grid cell size as a multiple of radius.</summary>
    public float GridCellRadiusScale { get; set; } = 2.25f;
}

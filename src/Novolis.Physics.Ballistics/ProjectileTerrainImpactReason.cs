namespace Novolis.Physics.Ballistics;

/// <summary>Reason a ballistic trajectory terminated.</summary>
public enum ProjectileTerrainImpactReason
{
    /// <summary>Hit static triangle mesh geometry.</summary>
    TerrainMesh,

    /// <summary>Hit heightfield or range-box terrain.</summary>
    Heightfield,

    /// <summary>Left the playable horizontal range.</summary>
    BeyondRange,

    /// <summary>Exceeded <see cref="ProjectileTerrainStepOptions.MaxSteps"/>.</summary>
    MaxSteps,
}

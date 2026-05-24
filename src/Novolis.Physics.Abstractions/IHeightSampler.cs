namespace Novolis.Physics.Abstractions;

/// <summary>Samples terrain surface height at world XZ (meters).</summary>
public interface IHeightSampler
{
    /// <summary>Returns surface height at world coordinates (<paramref name="x"/>, <paramref name="z"/>).</summary>
    float SampleHeight(float x, float z);
}

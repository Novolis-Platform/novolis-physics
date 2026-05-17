namespace Novolis.Physics.Abstractions;

/// <summary>Samples terrain surface height at world XZ (meters).</summary>
public interface IHeightSampler
{
    float SampleHeight(float x, float z);
}

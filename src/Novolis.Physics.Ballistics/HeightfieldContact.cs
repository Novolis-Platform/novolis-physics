using System.Numerics;
using Novolis.Physics.Abstractions;

namespace Novolis.Physics.Ballistics;

/// <summary>Heightfield contact tests against <see cref="IHeightSampler"/> and <see cref="AxisAlignedRangeBox"/>.</summary>
public static class HeightfieldContact
{
    /// <summary>TryContact operation.</summary>
    public static bool TryContact(
        IHeightSampler sampler,
        AxisAlignedRangeBox range,
        Vector3 position,
        float radius)
    {
        if (!range.IsInside(position.X, position.Z))
            return false;

        return position.Y <= sampler.SampleHeight(position.X, position.Z) + radius;
    }
/// <summary>ProjectOntoSurface operation.</summary>

    public static Vector3 ProjectOntoSurface(
        IHeightSampler sampler,
        AxisAlignedRangeBox range,
        Vector3 position,
        float surfaceEpsilon = 0.05f)
    {
        var x = System.Math.Clamp(position.X, 0f, range.ExtentMeters);
        var z = System.Math.Clamp(position.Z, 0f, range.ExtentMeters);
        var y = sampler.SampleHeight(x, z) + surfaceEpsilon;
        return new Vector3(x, y, z);
    }
}

namespace Novolis.Physics.Orbits;

/// <summary>Leapfrog kernel implementation for central-body acceleration.</summary>
public enum KernelMode
{
    /// <summary>Scalar inverse-square acceleration per body.</summary>
    Scalar,

    /// <summary>Uses <see cref="System.Numerics.Vector{T}"/> for central acceleration; requires hardware acceleration at test time.</summary>
    Vectorized,
}

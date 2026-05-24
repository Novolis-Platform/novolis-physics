namespace Novolis.Physics.Abstractions;

/// <summary>Advances <typeparamref name="TBody"/> given summed forces and torques for one fixed timestep.</summary>
public interface IIntegrator<TBody>
{
    /// <summary>Integrates <paramref name="body"/> forward by <paramref name="dtSeconds"/> using <paramref name="totalForcesAndTorques"/>.</summary>
    TBody Step(TBody body, in ForceSample totalForcesAndTorques, double dtSeconds);
}

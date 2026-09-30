using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Motion;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

static class TestBodyEnv
{
    public readonly record struct Body(Vector3 Position, double Mass);

    public readonly record struct Env(double Dummy);

    public sealed class ConstantForceModel(Vector3 force) : IForceModel<Body, Env>
    {
        public ForceSample Evaluate(Body body, Env environment, double timeSeconds) => new(force, Vector3.Zero);
    }

    public sealed class PointMassIntegrator : IIntegrator<Body>
    {
        public Body Step(Body body, in ForceSample totalForcesAndTorques, double dtSeconds)
        {
            var invM = 1.0 / body.Mass;
            var v = totalForcesAndTorques.Force.Multiply(invM * dtSeconds);
            return body with { Position = body.Position + v };
        }
    }
}

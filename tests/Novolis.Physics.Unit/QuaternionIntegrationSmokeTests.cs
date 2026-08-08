using Novolis.Physics.TestSupport;
using Novolis.Physics.Abstractions;
using Novolis.Physics.Motion;
using System.Numerics;
using Novolis.Math.Geometry;
using TUnit.Core;

namespace Novolis.Physics.Unit;

public sealed class QuaternionIntegrationSmokeTests
{
    [Test]
    public async Task ConstantTorque_QuaternionStaysNormalized_AndOmegaGrowsAlongAxis()
    {
        var integrator = new SemiImplicitEulerRigidBodyIntegrator();
        var torque = PhysicsTestVectors.V(0, 0, 0.6);
        var pipeline = new SimulationPipeline<RigidBodyState, int>(integrator, new ConstantTorqueWorldForce(torque));
        var body = new RigidBodyState(
            Vector3.Zero,
            Vector3.Zero,
            Quaternion.Identity,
            Vector3.Zero,
            mass: 1.0,
            inertiaDiagonalBody: PhysicsTestVectors.V(1, 2, 4));
        const double dt = 1.0 / 240.0;
        const int steps = 400;
        var samples = new List<SpinSampleRow>(capacity: 9);
        for (var i = 0; i < steps; i++)
        {
            body = pipeline.Step(body, 0, dt, i * dt);
            if (i % 100 == 0)
                samples.Add(new SpinSampleRow(i, body.AngularVelocity.Z, QuaternionNorm(body.Orientation)));
        }

        var o = NovolisPhysicsTestTrace.Out;
        PhysicsDashboard.ResultsAndTable(
            o,
            "Constant torque spin - samples",
            TestOutputSequences.EveryNth(samples, 1),
            new TableOptions { MaxCellWidth = 22, RightAlignNumericColumns = true },
            tableCaption: "world torque (0,0,0.6) Nm, Iz=4 kg m2; expect |q|~1, omega_z ~ alpha*t");

        await Assert.That(global::System.Math.Abs(QuaternionNorm(body.Orientation) - 1.0)).IsLessThanOrEqualTo((float)(1e-7));
        var alphaZ = torque.Z / body.InertiaDiagonalBody.Z;
        var expectedOmega = alphaZ * steps * dt;
        await Assert.That(global::System.Math.Abs(body.AngularVelocity.Z - expectedOmega)).IsLessThanOrEqualTo((float)(0.02 * global::System.Math.Max(1, global::System.Math.Abs(expectedOmega))));
    }

    private sealed class ConstantTorqueWorldForce(Vector3 torqueWorld) : IForceModel<RigidBodyState, int>
    {
        public ForceSample Evaluate(RigidBodyState body, int environment, double timeSeconds) =>
            new(Vector3.Zero, torqueWorld);
    }

    private sealed record SpinSampleRow(int Step, double OmegaZ, double OrientationNorm);

    private static double QuaternionNorm(Quaternion q) =>
        global::System.Math.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W);
}

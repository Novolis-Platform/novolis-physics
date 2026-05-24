using Novolis.Physics.Abstractions;
using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Motion;

/// <summary>Semi-implicit (symplectic) Euler for translation and diagonal body inertia with quaternion orientation.</summary>
public sealed class SemiImplicitEulerRigidBodyIntegrator : IIntegrator<RigidBodyState>
{
    /// <summary>Step operation.</summary>
    public RigidBodyState Step(RigidBodyState body, in ForceSample totalForcesAndTorques, double dtSeconds)
    {
        var invMass = body.Mass > 1e-30 ? 1.0 / body.Mass : 0;
        var dt = (float)dtSeconds;
        var accel = totalForcesAndTorques.Force * (float)invMass;
        var vel = body.Velocity + accel * dt;
        var pos = body.Position + vel * dt;

        var invI = new Vector3(
            body.InertiaDiagonalBody.X > 1e-30f ? (float)(1.0 / body.InertiaDiagonalBody.X) : 0f,
            body.InertiaDiagonalBody.Y > 1e-30f ? (float)(1.0 / body.InertiaDiagonalBody.Y) : 0f,
            body.InertiaDiagonalBody.Z > 1e-30f ? (float)(1.0 / body.InertiaDiagonalBody.Z) : 0f);

        var worldTau = totalForcesAndTorques.Torque;
        var bodyTau = InverseRotate(worldTau, body.Orientation);
        var bodyOmega = body.AngularVelocity;
        var bodyAlpha = new Vector3(bodyTau.X * invI.X, bodyTau.Y * invI.Y, bodyTau.Z * invI.Z);
        var newBodyOmega = bodyOmega + bodyAlpha * dt;
        var newOrientation = QuaternionAngularIntegration.IntegrateAngularVelocity(body.Orientation, newBodyOmega, dt);

        return new RigidBodyState(pos, vel, newOrientation, newBodyOmega, body.Mass, body.InertiaDiagonalBody);
    }

    private static Vector3 InverseRotate(Vector3 world, Quaternion orientation) =>
        QuaternionAngularIntegration.InverseRotate(world, orientation);
}

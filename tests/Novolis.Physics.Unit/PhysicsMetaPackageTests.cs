namespace Novolis.Physics.Unit;

public sealed class PhysicsMetaPackageTests
{
    [Test]
    public async Task AggregatePackage_BringsMotionAndGravityAssemblies()
    {
        await Assert.That(typeof(Novolis.Physics.Motion.SemiImplicitEulerRigidBodyIntegrator).Assembly.GetName().Name)
            .IsEqualTo("Novolis.Physics.Motion");
        await Assert.That(typeof(Novolis.Physics.Gravity.PointMassGravityModel).Assembly.GetName().Name)
            .IsEqualTo("Novolis.Physics.Gravity");
    }
}

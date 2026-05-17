using System.Numerics;
using Novolis.Math.Geometry;

namespace Novolis.Physics.Orbits;

/// <summary>Test-particle state in 3D; planar Earth orbit uses Z = 0 and Vz = 0.</summary>
public readonly record struct OrbitState(Vector3 Position, Vector3 Velocity);

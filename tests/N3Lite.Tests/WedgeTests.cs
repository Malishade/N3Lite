using System;
using N3Lite;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    /// <summary>
    /// A body dropped into a V between two steep faces: the pocket a character wedged itself in on
    /// playfield 4310 (two statel faces, normal Y 0.15 and 0.08, 0.66 m apart, 5 m above the terrain).
    /// Neither face passes the slope gate, so before the blocked-fall rule the body stayed airborne at
    /// terminal velocity without moving, and could not jump out.
    /// </summary>
    public class WedgeTests
    {
        sealed class Flat : ITileHeightSource
        {
            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => 0f;
        }

        const float CentreZ = 20f;
        const float Bottom = 7.7f;
        const float Top = 14f;

        /// <summary>Two faces running along X, leaning in so they meet at <see cref="Bottom"/>.</summary>
        static ISurface Pocket()
        {
            // The captured faces' lean: normal (0, 0.15, 0.989), so dz/dy = 0.15 / 0.989.
            float lean = 0.15f / 0.989f;
            float spread = (Top - Bottom) * lean;

            var v = new[]
            {
                // South face.
                new Vec3(10f, Bottom, CentreZ), new Vec3(30f, Bottom, CentreZ),
                new Vec3(30f, Top, CentreZ - spread), new Vec3(10f, Top, CentreZ - spread),
                // North face.
                new Vec3(10f, Bottom, CentreZ), new Vec3(30f, Bottom, CentreZ),
                new Vec3(30f, Top, CentreZ + spread), new Vec3(10f, Top, CentreZ + spread),
            };

            // Both windings, so either side of each face is solid.
            var idx = new[]
            {
                0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2,
                4, 5, 6, 4, 6, 7, 4, 6, 5, 4, 7, 6,
            };

            return new TilemapSurface(new Flat()) { Child = new TriangleMeshSurface(v, idx) };
        }

        static CharVehicleSim Body(ISurface surface, Vec3 at)
        {
            var sim = new CharVehicleSim
            {
                Mass = 50f,
                MaxForce = 10f,
                MaxVel = 1f,
                NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f,
                Surface = surface,
                Position = at,
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UpdateMotionConstraints(6f);
            return sim;
        }

        [Fact]
        public void AWedgedBodyStandsInThePocket()
        {
            var sim = Body(Pocket(), new Vec3(20f, 10f, CentreZ));

            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.False(sim.Airborne, "a body the faces hold up must count as standing");
            Assert.Equal(0f, sim.VerticalVelocity);
            Assert.InRange(sim.Position.Y, Bottom, 10.5f);
        }

        [Fact]
        public void TheWedgeDoesNotFlickerBetweenStandingAndFalling()
        {
            var sim = Body(Pocket(), new Vec3(20f, 10f, CentreZ));
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            int landed = 0;
            sim.JumpLanded += () => landed++;
            for (int i = 0; i < 120; i++)
            {
                sim.Run(1f / 60f);
                Assert.False(sim.Airborne);
            }

            Assert.Equal(0, landed);
        }

        [Fact]
        public void AWedgedBodyCanJump()
        {
            var sim = Body(Pocket(), new Vec3(20f, 10f, CentreZ));
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Jump(2f));
            Assert.True(sim.VerticalVelocity > 0f, "the launch must not be dropped");

            float start = sim.Position.Y;
            float peak = start;
            for (int i = 0; i < 30; i++)
            {
                sim.Run(1f / 60f);
                peak = MathF.Max(peak, sim.Position.Y);
            }

            Assert.True(peak > start + 1f);
        }

        [Fact]
        public void OpenAirStillFalls()
        {
            var sim = Body(Pocket(), new Vec3(40f, 10f, CentreZ));

            sim.Run(1f / 60f);
            for (int i = 0; i < 10; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Airborne);
            Assert.True(sim.Position.Y < 10f);
        }
    }
}

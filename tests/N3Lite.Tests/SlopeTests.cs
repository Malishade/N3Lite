using System;
using N3Lite;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class SlopeTests
    {
        sealed class Ramp : ITileHeightSource
        {
            readonly float _rise;
            readonly int _startX;

            public Ramp(float risePerTile, int startX = 10)
            {
                _rise = risePerTile;
                _startX = startX;
            }

            public int Width => 512;
            public int Height => 512;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => x <= _startX ? 0f : (x - _startX) * _rise;
        }

        static CharVehicleSim Walker(ITileHeightSource tiles, Vec3 at)
        {
            var sim = new CharVehicleSim
            {
                Mass = 50f,
                MaxForce = 10f,
                MaxVel = 1f,
                NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f,
                Surface = new TilemapSurface(tiles),
                Position = at,
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UpdateMotionConstraints(6f);
            return sim;
        }

        [Fact]
        public void AGentleSlopeIsWalkableAndTheBodyStaysGrounded()
        {
            var sim = Walker(new Ramp(0.3f), new Vec3(20.5f, 3.2f, 20.5f));

            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.False(sim.Airborne);
        }

        [Fact]
        public void ASteepSlopeLeavesTheBodyAirborneSoItSlides()
        {
            var sim = Walker(new Ramp(4f), new Vec3(20.5f, 42f, 20.5f));

            for (int i = 0; i < 30; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Airborne,
                "a face steeper than the gate must leave the body airborne, which is what makes it slide");
        }

        [Fact]
        public void TheGateIsNormalYOfAHalf()
        {
            var shallow = Walker(new Ramp(1.6f), new Vec3(20.5f, 17f, 20.5f));
            for (int i = 0; i < 120; i++)
                shallow.Run(1f / 60f);
            Assert.False(shallow.Airborne, "normal.y 0.53 should be walkable");

            var steep = Walker(new Ramp(1.9f), new Vec3(20.5f, 20f, 20.5f));
            for (int i = 0; i < 60; i++)
                steep.Run(1f / 60f);
            Assert.True(steep.Airborne, "normal.y 0.47 should not be walkable");
        }

        [Fact]
        public void RelaxingTheGateMakesASteepFaceWalkable()
        {
            var sim = Walker(new Ramp(4f), new Vec3(20.5f, 42f, 20.5f));
            sim.RelaxSlopeGate = true;

            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.False(sim.Airborne);
        }

        [Theory]
        [InlineData(0.2f, true)]
        [InlineData(0.5f, true)]
        [InlineData(1.0f, true)]
        [InlineData(1.7f, true)]
        [InlineData(1.9f, false)]
        [InlineData(4.0f, false)]
        [InlineData(10.0f, false)]
        [InlineData(20.0f, false)]
        public void OnlyWalkableSlopesCanBeClimbed(float rise, bool shouldClimb)
        {
            var sim = Walker(new Ramp(rise, startX: 25), new Vec3(20.5f, 0.01f, 20.5f));
            sim.UseSurfaceNormal();
            sim.SetForwardDrive(1f);
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(Math.PI / 2.0));

            for (int i = 0; i < 300; i++)
                sim.Run(1f / 60f);

            double ny = 1.0 / Math.Sqrt(1 + rise * rise);
            if (shouldClimb)
                Assert.True(sim.Position.Y > 4f,
                    $"rise {rise} (normal.y {ny:F3}) is walkable but the body only reached y={sim.Position.Y}");
            else
                Assert.True(sim.Position.Y < 3f,
                    $"rise {rise} (normal.y {ny:F3}) is not walkable but the body climbed to y={sim.Position.Y}");
        }

        [Fact]
        public void TheSurfaceNormalFromARayIsUnitLength()
        {
            var surface = new TilemapSurface(new Ramp(4f, 10));

            Assert.True(surface.GetLineIntersection(
                new Vec3(14.5f, 30f, 20.5f), new Vec3(14.5f, -30f, 20.5f),
                out _, out Vec3 normal, true, null));

            Assert.Equal(1f, normal.Length, 3);
        }

        [Fact]
        public void TheSurfaceNormalIsSquaredUpOnAnythingSteep()
        {
            var sim = Walker(new Ramp(4f), new Vec3(20.5f, 42f, 20.5f));
            sim.Run(1f / 60f);

            Assert.Equal(1f, sim.SurfaceNormal.Y, 3);
        }

        [Theory]
        [InlineData(1.9f)]
        [InlineData(2.5f)]
        [InlineData(4.0f)]
        [InlineData(10.0f)]
        [InlineData(20.0f)]
        public void HoldingForwardIntoAnUnwalkableSlopeMovesTheBodyNotOneMillimetre(float rise)
        {
            var sim = Walker(new Ramp(rise, startX: 25), new Vec3(20.5f, 0.01f, 20.5f));
            sim.UseSurfaceNormal();
            sim.SetForwardDrive(1f);
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(Math.PI / 2.0));

            float highest = sim.Position.Y;
            for (int i = 0; i < 600; i++)
            {
                sim.Run(1f / 60f);
                if (sim.Position.Y > highest)
                    highest = sim.Position.Y;
            }

            Assert.True(highest < 0.05f,
                $"rise {rise}: the body climbed to y={highest} (final y={sim.Position.Y}, x={sim.Position.X})");
        }

        [Fact]
        public void TheBodyComesToRestAgainstAnUnwalkableSlopeAndStopsMovingForward()
        {
            var sim = Walker(new Ramp(4f, startX: 25), new Vec3(20.5f, 0.01f, 20.5f));
            sim.UseSurfaceNormal();
            sim.SetForwardDrive(1f);
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(Math.PI / 2.0));

            for (int i = 0; i < 400; i++)
                sim.Run(1f / 60f);

            float restX = sim.Position.X;
            float restY = sim.Position.Y;

            for (int i = 0; i < 120; i++)
                sim.Run(1f / 60f);

            Assert.Equal(restX, sim.Position.X, 3);
            Assert.Equal(restY, sim.Position.Y, 3);
        }
    }
}

using System;
using System.Collections.Generic;
using N3Lite;
using N3Lite.Liquids;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class SwimTests
    {
        /// <summary>A lake bed at 0 up to x = 30, then a bank 4 high.</summary>
        sealed class Shore : ITileHeightSource
        {
            public int Width => 64;
            public int Height => 64;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => x <= 30 ? 0f : 4f;
        }

        const float Surface = 4.5f;

        /// <summary>Water at <see cref="Surface"/> over x 0..30, z 0..64, 10 deep.</summary>
        static TilemapSurface Lake()
        {
            var cells = new CellSurface(7, 7, 64f, 64f);
            var ground = new TilemapSurface(new Shore()) { Child = cells };
            var zones = new LiquidZones(p => cells.GetCellIdFromPos(p));
            var list = new List<LiquidTriangle>();
            int flags = 100 << 5;
            var a = new Vec3(0, Surface, 0);
            var b = new Vec3(30, Surface, 0);
            var c = new Vec3(30, Surface, 64);
            var d = new Vec3(0, Surface, 64);
            LiquidBuilder.Create(0, LiquidBuilder.DepthOf(flags), a, b, c, list);
            LiquidBuilder.Create(0, LiquidBuilder.DepthOf(flags), a, c, d, list);
            foreach (LiquidTriangle t in list)
                for (int cell = 0; cell < cells.CellCount; cell++)
                    zones.SetWater(cell, t);
            zones.CalculateWaterHeightMax();
            ground.Liquid = zones;
            return ground;
        }

        static CharVehicleSim Swimmer(Vec3 at)
        {
            var sim = new CharVehicleSim
            {
                Mass = 50f,
                MaxForce = 10f,
                MaxVel = 1f,
                NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f,
                Surface = Lake(),
                Position = at,
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UpdateMotionConstraints(6f);
            return sim;
        }

        [Fact]
        public void ADeepLakeHoldsTheBodyAtTheSurface()
        {
            CharVehicleSim sim = Swimmer(new Vec3(20.5f, 3f, 20.5f));
            int entered = 0;
            sim.EnteredLiquid += () => entered++;

            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.InLiquid);
            Assert.Equal(1, entered);
            Assert.Equal(Surface - 0.01f, sim.Position.Y, 3);
            Assert.Equal(0.01f, sim.LiquidDepth, 3);
        }

        [Fact]
        public void TheBodySwimsOverABankUnderTheSurfaceAndWalksOut()
        {
            CharVehicleSim sim = Swimmer(new Vec3(20.5f, 3f, 20.5f));
            int left = 0;
            sim.LeftLiquid += () => left++;
            sim.UseSurfaceNormal();
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(Math.PI / 2.0));

            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);
            sim.SetForwardDrive(1f);
            for (int i = 0; i < 1200 && sim.Position.X < 35f; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Position.X >= 35f, $"stuck at x={sim.Position.X}, y={sim.Position.Y}");
            Assert.False(sim.InLiquid);
            Assert.Equal(1, left);
            Assert.Equal(4.01f, sim.Position.Y, 2);
            Assert.Equal(VehicleSim.ClearOfLiquid, sim.LiquidDepth);
        }

        [Fact]
        public void ShallowWaterIsWadedNotSwum()
        {
            // Ground at 4 under a surface at 4.5: half a unit deep, under the 1.19 threshold.
            CharVehicleSim sim = Swimmer(new Vec3(40.5f, 4.2f, 20.5f));
            for (int i = 0; i < 60; i++)
                sim.Run(1f / 60f);
            Assert.False(sim.InLiquid);
            Assert.Equal(4.01f, sim.Position.Y, 2);
        }
    }
}

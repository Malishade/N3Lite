using System;
using System.Collections.Generic;
using N3Lite;
using N3Lite.Liquids;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class LiquidLiftTests
    {
        sealed class FlatHeights : ITileHeightSource
        {
            public FlatHeights(float y) { Y = y; }
            public float Y { get; }
            public int Width => 64;
            public int Height => 64;
            public float TileSize => 1f;
            public float SampleHeight(int x, int z) => Y;
        }

        static LiquidBuilder.MeshTriangle Tri(float y, int flags, float x0 = 0f, float z0 = 0f, float size = 40f)
            => new LiquidBuilder.MeshTriangle(
                new Vec3(x0, y, z0), new Vec3(x0 + size, y, z0), new Vec3(x0, y, z0 + size), flags);

        /// <summary>Ground at 0, a lake surface at <paramref name="surface"/> over (0..40, 0..40), depth 10.</summary>
        static TilemapSurface Lake(float surface)
        {
            var cells = new CellSurface(7, 7, 64f, 64f);
            var ground = new TilemapSurface(new FlatHeights(0f)) { Child = cells };
            var zones = new LiquidZones(p => cells.GetCellIdFromPos(p));
            var list = new List<LiquidTriangle>();
            LiquidBuilder.Create(Tri(surface, 100 << 5), list);
            foreach (LiquidTriangle t in list)
                for (int cell = 0; cell < cells.CellCount; cell++)
                    zones.SetWater(cell, t);
            zones.CalculateWaterHeightMax();
            ground.Liquid = zones;
            return ground;
        }

        [Fact]
        public void GroundDeepUnderWaterIsRaisedToSwimmingDepth()
        {
            TilemapSurface s = Lake(5f);
            var medium = new LiquidMedium();
            s.CalculateClosestPoint(new Vec3(5, 3, 5), out Vec3 closest, out Vec3 normal, null, medium);
            Assert.Equal((float)(5.0 - LiquidLift.Depth), closest.Y, 5);
            Assert.Equal(1f, normal.Y, 5);
            Assert.Equal(5f, medium.Height, 5);
        }

        [Fact]
        public void ShallowWaterLeavesTheGroundAlone()
        {
            TilemapSurface s = Lake(1f);
            s.CalculateClosestPoint(new Vec3(5, 0.5f, 5), out Vec3 closest, out _, null);
            Assert.Equal(0f, closest.Y);
        }

        [Fact]
        public void AwayFromTheLakeNothingChanges()
        {
            TilemapSurface s = Lake(5f);
            s.CalculateClosestPoint(new Vec3(50, 3, 50), out Vec3 closest, out _, null);
            Assert.Equal(0f, closest.Y);
        }

        [Fact]
        public void RoomLiquidSkipsSlopedTypesAndKeepsTheRest()
        {
            var room = new RoomLiquid(new[] { Tri(2f, 1), Tri(2f, 0), Tri(2f, 4 | (3 << 5)) });
            Assert.Equal(2, room.Count);
        }

        [Fact]
        public void RoomDepthReadsTheHighBitsUnsigned()
        {
            Assert.Equal(LiquidBuilder.NoDepth, RoomLiquid.DepthOf(4));
            Assert.Equal(0.3f, RoomLiquid.DepthOf(3 << 5), 6);
            // Unsigned: 0x80000000 >> 5 is 67108864, so a depth of 6710886.4 rather than a negative one.
            Assert.Equal(6710886.4f, RoomLiquid.DepthOf(unchecked((int)0x80000000)));
        }

        [Fact]
        public void TheRoomTestWorksInRoomSpace()
        {
            // A room at (100, 0, 200) unturned: both the triangle and the point's offset from the origin
            // get (1000, 0, 1000), so a triangle at room (0..40) covers world (100..140, 200..240).
            var room = new RoomLiquid(new[] { Tri(2f, 5 << 5) });
            var origin = new Vec3(100, 0, 200);
            float h = room.SurfaceHeight(new Vec3(105, 1, 205), origin, 0, out Vec3 n, out int type, out float depth);
            Assert.Equal(2f, h, 5);
            Assert.Equal(0.5f, depth, 6);
            Assert.Equal(1f, n.Y, 5);
            Assert.Equal(origin.Y - RoomLiquid.MissDepth, room.SurfaceHeight(new Vec3(150, 1, 205), origin, 0, out _, out _, out _));
        }

        [Fact]
        public void ATurnedRoomTurnsThePointTheOtherWay()
        {
            // One quarter turn: the point's offset is turned by -90 degrees, (dx, dz) -> (-dz, dx), so an
            // offset of (5, -5) lands at room (5, 5), inside the triangle.
            var room = new RoomLiquid(new[] { Tri(2f, 5 << 5) });
            var origin = new Vec3(100, 0, 200);
            float h = room.SurfaceHeight(new Vec3(105, 1, 195), origin, 1, out _, out _, out _);
            Assert.Equal(2f, h, 4);
        }

        [Fact]
        public void TheRoomCollisionTestNeedsTheBottomAtOrUnderTheLimit()
        {
            var room = new RoomLiquid(new[] { Tri(2f, 5 << 5) });
            var origin = new Vec3(100, 0, 200);
            var p = new Vec3(105, 1, 205);
            Assert.Equal(2f, room.PerformLiquidCollisionTest(p, out _, out _, 1.5f, origin, 0), 5);
            Assert.Equal(LiquidZones.NoLiquid, room.PerformLiquidCollisionTest(p, out _, out _, 1.4f, origin, 0));
        }
    }
}

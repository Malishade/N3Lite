using System.Collections.Generic;
using N3Lite;
using N3Lite.Liquids;
using Xunit;

namespace N3Lite.Tests
{
    public class LiquidTests
    {
        static List<LiquidTriangle> Build(int type, float depth, Vec3 a, Vec3 b, Vec3 c)
        {
            var list = new List<LiquidTriangle>();
            LiquidBuilder.Create(type, depth, a, b, c, list);
            return list;
        }

        /// <summary>One zone for everything, for tests that are not about zones.</summary>
        static LiquidZones OneZone(List<LiquidTriangle> triangles)
        {
            var zones = new LiquidZones(_ => 0);
            foreach (LiquidTriangle t in triangles)
                zones.SetWater(0, t);
            zones.CalculateWaterHeightMax();
            return zones;
        }

        [Fact]
        public void ASmallFlatTriangleIsOneCollisionTriangle()
        {
            List<LiquidTriangle> t = Build(0, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150));
            Assert.Single(t);
            Assert.Equal(10f, t[0].SurfaceHeight(new Vec3(110, 0, 110)), 4);
        }

        [Fact]
        public void TheNormalAlwaysFacesUp()
        {
            // (50, 0, 0) x (0, 0, 50) faces down (Y = -2500), so v1 and v2 are swapped.
            List<LiquidTriangle> t = Build(0, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150));
            Assert.Single(t);
            Assert.Equal(1f, t[0].Normal.Y, 6);
            Assert.Equal(new Vec3(100, 10, 150), t[0].V1);
            Assert.Equal(new Vec3(150, 10, 100), t[0].V2);
            Assert.Equal(10f, t[0].SurfaceHeight(new Vec3(110, 0, 110)), 4);
        }

        [Fact]
        public void ALargeTriangleIsCutIntoFour()
        {
            // Twice the area is 120 x 120 = 14400 > 10000; each quarter is 3600.
            List<LiquidTriangle> t = Build(0, 5f, new Vec3(100, 10, 100), new Vec3(220, 10, 100), new Vec3(100, 10, 220));
            Assert.Equal(4, t.Count);
            // The corner pieces first (v0, v1, v2), then the middle one.
            Assert.Equal(new Vec3(100, 10, 100), t[0].V0);
            Assert.Equal(new Vec3(220, 10, 100), t[1].V0);
            Assert.Equal(new Vec3(100, 10, 220), t[2].V0);
            Assert.Equal(new Vec3(160, 10, 100), t[3].V0);
        }

        [Fact]
        public void SlopedLiquidTypesHaveNoCollision()
        {
            Assert.Empty(Build(1, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150)));
            Assert.Empty(Build(3, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150)));
        }

        [Fact]
        public void TrianglesOffTheMapAreDropped()
        {
            // The centre's cell, trunc(x / 100) - 60 * trunc(z / -100), must be 0..3599.
            Assert.Empty(Build(0, 5f, new Vec3(-200, 10, 10), new Vec3(-150, 10, 10), new Vec3(-200, 10, 60)));
            Assert.Empty(Build(0, 5f, new Vec3(10, 10, 6020), new Vec3(60, 10, 6020), new Vec3(10, 10, 6070)));
            // A centre just below x = 0 still truncates into column 0.
            Assert.Single(Build(0, 5f, new Vec3(-60, 10, 10), new Vec3(-10, 10, 10), new Vec3(-60, 10, 60)));
        }

        [Fact]
        public void RepeatedCornersMakeNothing()
        {
            Assert.Empty(Build(0, 5f, new Vec3(1, 1, 1), new Vec3(1, 1, 1), new Vec3(2, 1, 5)));
        }

        [Fact]
        public void TheEdgesAndBoundsAreStrict()
        {
            List<LiquidTriangle> t = Build(0, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150));
            Assert.Equal(0f, t[0].SurfaceHeight(new Vec3(100, 0, 120)));
            Assert.Equal(0f, t[0].SurfaceHeight(new Vec3(140, 0, 140)));
            // At or above the highest corner is not under the surface.
            Assert.Equal(0f, t[0].SurfaceHeight(new Vec3(110, 10, 110)));
        }

        [Fact]
        public void TheQueryFindsTheSurfaceAndItsType()
        {
            LiquidZones zones = OneZone(Build(4, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150)));
            var p = new Vec3(110, 8, 110);
            float h = zones.GetLiquidSurfaceHeight(ref p, out Vec3 n, out int type);
            Assert.Equal(10f, h, 4);
            Assert.Equal(4, type);
            Assert.True(n.Y > 0.99f);
        }

        [Fact]
        public void APointBelowTheBottomFindsNothing()
        {
            // Depth 5 under a surface at 10: the bottom is at 5, and the point's own height is the limit.
            LiquidZones zones = OneZone(Build(0, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150)));
            var p = new Vec3(110, 4, 110);
            Assert.Equal(LiquidZones.NoLiquid, zones.GetLiquidSurfaceHeight(ref p, out _, out _));
            p = new Vec3(110, 5, 110);
            Assert.Equal(10f, zones.GetLiquidSurfaceHeight(ref p, out _, out _), 4);
        }

        [Fact]
        public void AbovePlayfieldAndEmptyZoneSentinels()
        {
            LiquidZones zones = OneZone(Build(0, 5f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150)));
            var p = new Vec3(110, 11, 110);
            Assert.Equal(LiquidZones.AbovePlayfield, zones.GetLiquidSurfaceHeight(ref p, out _, out _));
            p = new Vec3(300, 5, 300);
            Assert.Equal(LiquidZones.NoLiquid, zones.GetLiquidSurfaceHeight(ref p, out _, out _));
        }

        [Fact]
        public void TheFirstTriangleOverThePointDecides()
        {
            // A shallow sheet filed first hides a deep one under it for a point below the shallow bottom.
            var list = new List<LiquidTriangle>();
            LiquidBuilder.Create(0, 1f, new Vec3(100, 10, 100), new Vec3(150, 10, 100), new Vec3(100, 10, 150), list);
            LiquidBuilder.Create(0, 50f, new Vec3(100, 9, 100), new Vec3(150, 9, 100), new Vec3(100, 9, 150), list);
            LiquidZones zones = OneZone(list);
            var p = new Vec3(110, 5, 110);
            Assert.Equal(LiquidZones.NoLiquid, zones.GetLiquidSurfaceHeight(ref p, out _, out _));
        }

        [Fact]
        public void TheSweepFilesATriangleWithEveryZoneNearIt()
        {
            // 10 x 10 tiles of 10 units, zones of 2 tiles (20 units): zone = (int)(x / 20) + (int)(z / 20) * 5.
            var zones = new LiquidZones(p => (int)(p.X / 20f) + (int)(p.Z / 20f) * 5);
            var list = new List<LiquidTriangle>();
            LiquidBuilder.Create(0, 5f, new Vec3(45, 10, 45), new Vec3(55, 10, 45), new Vec3(45, 10, 55), list);
            zones.AddWater(list, 10f, 10, 10, 2);
            // One step either side of 45..55: x and z run 25, 45, 65 -> zones 1..3 in each direction.
            Assert.Equal(1, zones.CountIn(6));
            Assert.Equal(1, zones.CountIn(12));
            Assert.Equal(1, zones.CountIn(18));
            Assert.Equal(0, zones.CountIn(0));
            Assert.Equal(10f, zones.HeightOf(12));
            Assert.Equal(-1f, zones.HeightOf(0));
        }

        [Fact]
        public void DepthComesFromTheFlagsInTenths()
        {
            Assert.Equal(0.5f, LiquidBuilder.DepthOf(5 << 5), 6);
            Assert.Equal(LiquidBuilder.NoDepth, LiquidBuilder.DepthOf(3));
            Assert.Equal(18, LiquidBuilder.TypeOf(18 | (7 << 5)));
        }

        [Fact]
        public void APolygonMeshIsAFanRoundItsCentre()
        {
            var v = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(10, 0, 0), new Vec3(10, 0, 10), new Vec3(0, 0, 10) };
            List<LiquidBuilder.MeshTriangle> t = LiquidBuilder.ReadMeshes(new[] { new LiquidBuilder.Mesh(0, v, new int[0]) });
            Assert.Equal(4, t.Count);
            Assert.Equal(new Vec3(5, 0, 5), t[0].V0);
            Assert.Equal(v[3], t[0].V1);
            Assert.Equal(v[0], t[0].V2);
            Assert.Equal(v[0], t[1].V1);
            Assert.Equal(v[1], t[1].V2);
        }

        [Fact]
        public void AnIndexedMeshKeepsItsTrianglesInOrderUpToTheCap()
        {
            var v = new List<Vec3> { new Vec3(0, 0, 0), new Vec3(10, 0, 0), new Vec3(10, 0, 10), new Vec3(0, 0, 10) };
            // Flag bit 0: indexed. Four corners allow 4 - 2 = 2 triangles; the third is dropped.
            var idx = new[] { 0, 1, 2, 0, 2, 3, 1, 2, 3 };
            List<LiquidBuilder.MeshTriangle> t = LiquidBuilder.ReadMeshes(new[] { new LiquidBuilder.Mesh(1, v, idx) });
            Assert.Equal(2, t.Count);
            Assert.Equal(v[0], t[1].V0);
            Assert.Equal(v[2], t[1].V1);
            Assert.Equal(v[3], t[1].V2);
        }
    }
}

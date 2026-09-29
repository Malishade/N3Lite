using System;
using N3Lite;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class TilemapSurfaceTests
    {
        sealed class FuncHeights : ITileHeightSource
        {
            readonly Func<int, int, float> _height;

            public FuncHeights(int width, int height, float tileSize, Func<int, int, float> f)
            {
                Width = width;
                Height = height;
                TileSize = tileSize;
                _height = f;
            }

            public int Width { get; }
            public int Height { get; }
            public float TileSize { get; }
            public float SampleHeight(int x, int z) => _height(x, z);
        }

        static TilemapSurface Flat(float y, float tileSize = 1f, int extent = 64)
            => new TilemapSurface(new FuncHeights(extent, extent, tileSize, (_, _) => y));

        [Fact]
        public void ADownwardRayOntoFlatGroundHitsAtTheGroundHeight()
        {
            TilemapSurface surface = Flat(10f);

            bool hit = surface.GetLineIntersection(
                new Vec3(4.3f, 12f, 7.1f),
                new Vec3(4.3f, 0f, 7.1f),
                out Vec3 point, out Vec3 normal, true, null);

            Assert.True(hit);
            Assert.Equal(10f, point.Y, 4);
            Assert.Equal(4.3f, point.X, 4);
            Assert.Equal(7.1f, point.Z, 4);
        }

        [Fact]
        public void TheNormalHandedBackOpposesTheRay()
        {
            TilemapSurface surface = Flat(0f);

            Assert.True(surface.GetLineIntersection(
                new Vec3(2.5f, 5f, 2.5f), new Vec3(2.5f, -5f, 2.5f),
                out _, out Vec3 normal, true, null));

            float length = normal.Length;
            Assert.True(length > 0f);
            Assert.Equal(1f, normal.Y / length, 3);
        }

        [Fact]
        public void ARayThatStopsAboveTheGroundDoesNotHit()
        {
            TilemapSurface surface = Flat(0f);

            Assert.False(surface.GetLineIntersection(
                new Vec3(2.5f, 10f, 2.5f), new Vec3(2.5f, 5f, 2.5f),
                out _, out _, true, null));
        }

        [Fact]
        public void AMissWritesTheMinusOneSentinel()
        {
            TilemapSurface surface = Flat(0f);

            Assert.False(surface.GetLineIntersection(
                new Vec3(2.5f, 10f, 2.5f), new Vec3(2.5f, 5f, 2.5f),
                out Vec3 point, out _, true, null));

            Assert.Equal(new Vec3(-1f, -1f, -1f), point);
        }

        [Fact]
        public void AnUpwardRayFromBelowMissesBecauseTheTestIsOneSided()
        {
            TilemapSurface surface = Flat(0f);

            Assert.False(surface.GetLineIntersection(
                new Vec3(2.5f, -5f, 2.5f), new Vec3(2.5f, 5f, 2.5f),
                out _, out _, true, null));
        }

        [Fact]
        public void ARampIsSampledAtTheRightHeight()
        {
            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, (x, _) => x));

            Assert.True(surface.GetLineIntersection(
                new Vec3(5f, 20f, 3f), new Vec3(5f, -1f, 3f),
                out Vec3 point, out Vec3 normal, true, null));

            Assert.Equal(5f, point.Y, 3);
            Assert.True(normal.Y > 0f);
            Assert.True(normal.X < 0f);
        }

        [Fact]
        public void HeightIsInterpolatedAcrossATile()
        {
            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, (x, _) => x));

            Assert.True(surface.GetLineIntersection(
                new Vec3(5.5f, 20f, 3.25f), new Vec3(5.5f, -1f, 3.25f),
                out Vec3 point, out _, true, null));

            Assert.Equal(5.5f, point.Y, 3);
        }

        [Fact]
        public void TheDiagonalIsACheckerboardOfTileParity()
        {
            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, (_, _) => 0f));

            surface.GetTileCorners(0, 0, out _, out _, out _, out _, out int d00);
            surface.GetTileCorners(1, 1, out _, out _, out _, out _, out int d11);
            surface.GetTileCorners(1, 0, out _, out _, out _, out _, out int d10);
            surface.GetTileCorners(0, 1, out _, out _, out _, out _, out int d01);

            Assert.Equal(1, d00);
            Assert.Equal(1, d11);
            Assert.Equal(0, d10);
            Assert.Equal(0, d01);
        }

        [Fact]
        public void TheDiagonalDecidesWhichTriangleAProbeLandsOn()
        {
            static float Bump(int x, int z) => (x == 0 && z == 1) ? 4f : 0f;

            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, Bump));

            Assert.True(surface.IntersectTile(0, 0,
                new Vec3(0.8f, 10f, 0.2f), new Vec3(0.8f, -10f, 0.2f),
                out Vec3 below, out _));
            Assert.Equal(0f, below.Y, 3);

            Assert.True(surface.IntersectTile(0, 0,
                new Vec3(0.2f, 10f, 0.8f), new Vec3(0.2f, -10f, 0.8f),
                out Vec3 above, out _));
            Assert.Equal(2.4f, above.Y, 3);
        }

        [Fact]
        public void ADiagonalZeroTileSplitsTheOtherWay()
        {
            static float Bump(int x, int z) => (x == 1 && z == 0) ? 4f : 0f;

            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, Bump));

            Assert.True(surface.IntersectTile(1, 0,
                new Vec3(1.2f, 10f, 0.2f), new Vec3(1.2f, -10f, 0.2f),
                out Vec3 near, out _));
            Assert.Equal(2.4f, near.Y, 3);

            Assert.True(surface.IntersectTile(1, 0,
                new Vec3(1.8f, 10f, 0.8f), new Vec3(1.8f, -10f, 0.8f),
                out Vec3 far, out _));
            Assert.Equal(0f, far.Y, 3);
        }

        [Fact]
        public void AnAxisAlignedRayStillWalks()
        {
            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, (x, _) => x >= 10 ? 6f : 0f));

            Assert.True(surface.GetLineIntersection(
                new Vec3(2f, 5f, 4.5f), new Vec3(14f, 4f, 4.5f),
                out Vec3 point, out _, true, null));
            Assert.True(point.X > 8f, $"expected to reach the rise, stopped at x={point.X}");
        }

        [Fact]
        public void AShallowRayWalksAcrossTilesUntilItMeetsAWall()
        {
            static float Wall(int x, int z) => x >= 10 && x <= 11 ? 6f : 0f;
            var surface = new TilemapSurface(new FuncHeights(64, 64, 1f, Wall));

            bool hit = surface.GetLineIntersection(
                new Vec3(2f, 5f, 4.5f), new Vec3(14f, 4f, 4.5f),
                out Vec3 point, out _, true, null);

            Assert.True(hit);
            Assert.True(point.X >= 9.5f && point.X <= 12.5f, $"walked to x={point.X}");
        }

        [Fact]
        public void ARayEntirelyBelowTheTerrainFindsNothing()
        {
            TilemapSurface surface = Flat(50f);

            Assert.False(surface.GetLineIntersection(
                new Vec3(1f, 0f, 1f), new Vec3(20f, 0f, 20f),
                out _, out _, true, null));
        }

        [Fact]
        public void TilesOutsideTheMapAreSkipped()
        {
            TilemapSurface surface = Flat(0f, tileSize: 1f, extent: 8);

            Assert.False(surface.GetLineIntersection(
                new Vec3(40f, 5f, 40f), new Vec3(40f, -5f, 40f),
                out _, out _, true, null));
        }

        sealed class ConstantSurface : ISurface
        {
            readonly Vec3 _hit;
            readonly bool _result;

            public ConstantSurface(bool result, Vec3 hit)
            {
                _result = result;
                _hit = hit;
            }

            public bool GetLineIntersection(Vec3 start, Vec3 end, out Vec3 hit, out Vec3 normal, bool clip, object locality)
            {
                hit = _hit;
                normal = Vec3.ReferenceUp;
                return _result;
            }

            public bool VetoPosition(ref Vec3 position, object locality, Vec3 previous) => false;

            public void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality)
            {
                closest = _hit;
                normal = Vec3.ReferenceUp;
            }
        }

        [Fact]
        public void TheNearerOfTheChildAndTheTerrainWins()
        {
            TilemapSurface surface = Flat(0f);
            surface.Child = new ConstantSurface(true, new Vec3(2.5f, 3f, 2.5f));

            Assert.True(surface.GetLineIntersection(
                new Vec3(2.5f, 10f, 2.5f), new Vec3(2.5f, -5f, 2.5f),
                out Vec3 point, out _, true, null));

            Assert.Equal(3f, point.Y, 4);
        }

        [Fact]
        public void TheTerrainWinsWhenItIsNearerThanTheChild()
        {
            TilemapSurface surface = Flat(0f);
            surface.Child = new ConstantSurface(true, new Vec3(2.5f, -4f, 2.5f));

            Assert.True(surface.GetLineIntersection(
                new Vec3(2.5f, 10f, 2.5f), new Vec3(2.5f, -5f, 2.5f),
                out Vec3 point, out _, true, null));

            Assert.Equal(0f, point.Y, 4);
        }

        [Fact]
        public void TheChildStillCountsWhenTheTerrainMisses()
        {
            TilemapSurface surface = Flat(-100f);
            surface.Child = new ConstantSurface(true, new Vec3(2.5f, 1f, 2.5f));

            Assert.True(surface.GetLineIntersection(
                new Vec3(2.5f, 10f, 2.5f), new Vec3(2.5f, 0f, 2.5f),
                out Vec3 point, out _, true, null));

            Assert.Equal(1f, point.Y, 4);
        }

        [Fact]
        public void TheDirectHeightQueryAgreesWithTheRayCastOnWarpedTerrain()
        {
            var rng = new Random(20260923);
            var heights = new float[40, 40];
            for (int x = 0; x < 40; x++)
                for (int z = 0; z < 40; z++)
                    heights[x, z] = (float)(rng.NextDouble() * 8.0);

            var surface = new TilemapSurface(
                new FuncHeights(40, 40, 1f, (x, z) => heights[Math.Min(x, 39), Math.Min(z, 39)]));

            int compared = 0;
            for (int i = 0; i < 4000; i++)
            {
                float px = 1f + (float)(rng.NextDouble() * 36.0);
                float pz = 1f + (float)(rng.NextDouble() * 36.0);

                float direct = surface.GroundHeightAt(new Vec3(px, 0f, pz), out Vec3 directNormal);

                Assert.True(surface.GetLineIntersection(
                    new Vec3(px, 40f, pz), new Vec3(px, -40f, pz),
                    out Vec3 cast, out Vec3 castNormal, true, null));

                Assert.True(Math.Abs(direct - cast.Y) < 1e-3f,
                    $"at ({px:F3},{pz:F3}) direct={direct} cast={cast.Y}");

                float dn = directNormal.Length;
                float cn = castNormal.Length;
                if (dn > 0f && cn > 0f)
                {
                    float alignment = Vec3.Dot(directNormal * (1f / dn), castNormal * (1f / cn));
                    Assert.True(alignment > 0.9999f,
                        $"at ({px:F3},{pz:F3}) normals disagree: {directNormal} vs {castNormal}");
                }

                compared++;
            }

            Assert.Equal(4000, compared);
        }

        [Fact]
        public void TheDirectQueryReturnsZeroOutsideTheMap()
        {
            TilemapSurface surface = Flat(5f, tileSize: 1f, extent: 8);

            Assert.Equal(0f, surface.GroundHeightAt(new Vec3(-1f, 0f, 4f), out _), 4);
            Assert.Equal(0f, surface.GroundHeightAt(new Vec3(4f, 0f, -1f), out _), 4);
            Assert.Equal(0f, surface.GroundHeightAt(new Vec3(8f, 0f, 4f), out _), 4);
            Assert.Equal(0f, surface.GroundHeightAt(new Vec3(4f, 0f, 8f), out _), 4);
            Assert.Equal(5f, surface.GroundHeightAt(new Vec3(4f, 0f, 4f), out _), 4);
        }

        [Fact]
        public void TheChunkedSourceIndexesTheSameWayTheClientDoes()
        {
            const int side = 5;
            var a = new ushort[side, side];
            var b = new ushort[side, side];
            for (int x = 0; x < side; x++)
                for (int z = 0; z < side; z++)
                {
                    a[x, z] = (ushort)(100 + x + z * 10);
                    b[x, z] = (ushort)(200 + x + z * 10);
                }

            var source = new ChunkedTileHeightSource(
                new[] { a, b }, gridWidth: 2, chunkSide: side, heightMod: 0.5f,
                width: 8, height: 4, tileSize: 1f);

            Assert.Equal((100 + 1 + 20) * 0.5f, source.SampleHeight(1, 2), 4);
            Assert.Equal((200 + 1 + 20) * 0.5f, source.SampleHeight(5, 2), 4);
        }

        [Fact]
        public void AChunkSideThatIsNotAPowerOfTwoPlusOneIsRejected()
        {
            var chunk = new ushort[6, 6];
            Assert.Throws<ArgumentException>(() => new ChunkedTileHeightSource(
                new[] { chunk }, 1, 6, 1f, 6, 6, 1f));
        }
    }
}

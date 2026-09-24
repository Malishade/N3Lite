using System.Collections.Generic;
using N3Lite;
using N3Lite.Surfaces;
using Xunit;

namespace N3Lite.Tests
{
    public class CellSurfaceTests
    {
        sealed class Recorder : ICellWorker
        {
            public readonly List<int> Cells = new List<int>();
            public void DoCell(int cellId) => Cells.Add(cellId);
        }

        [Fact]
        public void TheWalkVisitsOneCellForALineInsideIt()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(15f, 0f, 15f), new Vec3(25f, 0f, 25f), r, 10, 10, 400f, 400f);

            Assert.Equal(new[] { 0 }, r.Cells);
        }

        [Fact]
        public void TheWalkCrossesCellsInOrderAlongTheLine()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(20f, 0f, 20f), new Vec3(140f, 0f, 20f), r, 10, 10, 400f, 400f);

            Assert.Equal(new[] { 0, 1, 2, 3 }, r.Cells);
        }

        [Fact]
        public void TheWalkGoesBackwardsToo()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(140f, 0f, 20f), new Vec3(20f, 0f, 20f), r, 10, 10, 400f, 400f);

            Assert.Equal(new[] { 3, 2, 1, 0 }, r.Cells);
        }

        [Fact]
        public void TheWalkStepsRowsWithCellsXStride()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(20f, 0f, 20f), new Vec3(20f, 0f, 100f), r, 10, 10, 400f, 400f);

            Assert.Equal(new[] { 0, 10, 20 }, r.Cells);
        }

        [Fact]
        public void AnOutOfRangeCellEndsTheWalkRatherThanBeingSkipped()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(-50f, 0f, 20f), new Vec3(140f, 0f, 20f), r, 10, 10, 400f, 400f);

            Assert.Empty(r.Cells);
        }

        [Fact]
        public void TheWalkStopsAtTheGridEdge()
        {
            var r = new Recorder();
            GridSpace.IterateCellsAlongLine(
                new Vec3(340f, 0f, 20f), new Vec3(600f, 0f, 20f), r, 10, 10, 400f, 400f);

            Assert.Equal(new[] { 8, 9 }, r.Cells);
        }

        static CellSurface Grid() => new CellSurface(10, 10, 400f, 400f);

        [Fact]
        public void CellIdsAreRowMajorAndOutOfRangeIsMinusOne()
        {
            CellSurface g = Grid();
            Assert.Equal(40f, g.CellSizeX);
            Assert.Equal(0, g.GetCellIdFromPos(new Vec3(20f, 0f, 20f)));
            Assert.Equal(1, g.GetCellIdFromPos(new Vec3(60f, 0f, 20f)));
            Assert.Equal(10, g.GetCellIdFromPos(new Vec3(20f, 0f, 60f)));
            Assert.Equal(99, g.GetCellIdFromPos(new Vec3(399f, 0f, 399f)));
            Assert.Equal(-1, g.GetCellIdFromPos(new Vec3(401f, 0f, 20f)));
            Assert.Equal(-1, g.GetCellIdFromPos(new Vec3(-41f, 0f, 20f)));

            Assert.Equal(0, g.GetCellIdFromPos(new Vec3(-1f, 0f, 20f)));
        }

        static TriangleMeshSurface Slab(float minX, float minZ, float size, float y)
        {
            var v = new[]
            {
                new Vec3(minX, y, minZ),
                new Vec3(minX + size, y, minZ),
                new Vec3(minX + size, y, minZ + size),
                new Vec3(minX, y, minZ + size),
            };
            return new TriangleMeshSurface(v, new[] { 0, 2, 1, 0, 3, 2 });
        }

        [Fact]
        public void ARayFindsGeometryInACellItCrosses()
        {
            CellSurface g = Grid();
            g.SetSurfaceForCell(0, Slab(0f, 0f, 40f, 5f));

            Assert.True(g.GetLineIntersection(
                new Vec3(20f, 20f, 20f), new Vec3(20f, -20f, 20f), out Vec3 hit, out Vec3 n, true, null));
            Assert.Equal(5f, hit.Y, 3);
            Assert.Equal(1f, n.Y, 3);
        }

        [Fact]
        public void AHitOutsideTheCellIsRejectedSoTheRightCellReportsIt()
        {
            CellSurface g = Grid();
            TriangleMeshSurface wide = Slab(0f, 0f, 80f, 5f);
            g.SetSurfaceForCell(0, wide);
            g.SetSurfaceForCell(1, wide);

            Assert.True(g.GetLineIntersection(
                new Vec3(60f, 20f, 20f), new Vec3(60f, -20f, 20f), out Vec3 hit, out _, true, null));
            Assert.Equal(60f, hit.X, 3);
        }

        [Fact]
        public void GeometryOnlyInAnotherCellIsNotReported()
        {
            CellSurface g = Grid();
            g.SetSurfaceForCell(0, Slab(0f, 0f, 80f, 5f));

            Assert.False(g.GetLineIntersection(
                new Vec3(60f, 20f, 20f), new Vec3(60f, -20f, 20f), out _, out _, true, null));
        }

        [Fact]
        public void TheNearestOfSeveralSurfacesInOneCellWins()
        {
            CellSurface g = Grid();
            g.SetSurfaceForCell(0, Slab(0f, 0f, 40f, 1f));
            g.SetSurfaceForCell(0, Slab(0f, 0f, 40f, 9f));

            Assert.True(g.GetLineIntersection(
                new Vec3(20f, 20f, 20f), new Vec3(20f, -20f, 20f), out Vec3 hit, out _, true, null));
            Assert.Equal(9f, hit.Y, 3);
        }

        [Fact]
        public void TheWalkStopsAtTheFirstCellWithAHit()
        {
            CellSurface g = Grid();
            g.SetSurfaceForCell(1, Wall(60f));
            g.SetSurfaceForCell(2, Wall(100f));

            Assert.True(g.GetLineIntersection(
                new Vec3(20f, 5f, 20f), new Vec3(140f, 5f, 20f), out Vec3 hit, out _, true, null));
            Assert.Equal(60f, hit.X, 3);
        }

        static TriangleMeshSurface Wall(float x)
        {
            var v = new[]
            {
                new Vec3(x, 0f, 0f),
                new Vec3(x, 0f, 400f),
                new Vec3(x, 20f, 400f),
                new Vec3(x, 20f, 0f),
            };
            return new TriangleMeshSurface(v, new[] { 0, 1, 2, 0, 2, 3 });
        }

        [Fact]
        public void AnEmptyGridReportsNothing()
        {
            Assert.False(Grid().GetLineIntersection(
                new Vec3(20f, 20f, 20f), new Vec3(20f, -20f, 20f), out _, out _, true, null));
        }

        [Fact]
        public void TheClosestPointComesFromTheCellAtThePosition()
        {
            CellSurface g = Grid();
            g.SetSurfaceForCell(0, Slab(0f, 0f, 40f, 5f));

            g.CalculateClosestPoint(new Vec3(20f, 6f, 20f), out Vec3 c, out Vec3 n, null);

            Assert.Equal(5f, c.Y, 3);
            Assert.Equal(1f, n.Y, 3);
        }

        [Fact]
        public void AnEmptyCellAnswersWithTheNoClosestPointSentinel()
        {
            var p = new Vec3(20f, 6f, 20f);
            Grid().CalculateClosestPoint(p, out Vec3 c, out _, null);
            Assert.Equal(TilemapSurface.NoClosestPoint, c.Y, 3);
        }

        [Fact]
        public void AMeshNormalFacesTheCaller()
        {
            TriangleMeshSurface wall = Wall(10f);

            Assert.True(wall.GetLineIntersection(
                new Vec3(0f, 5f, 200f), new Vec3(20f, 5f, 200f), out _, out Vec3 n, true, null));
            Assert.Equal(-1f, n.X, 3);
        }

        [Fact]
        public void AMeshIsSolidFromOneSideOnly()
        {
            TriangleMeshSurface wall = Wall(10f);

            Assert.False(wall.GetLineIntersection(
                new Vec3(20f, 5f, 200f), new Vec3(0f, 5f, 200f), out _, out _, true, null));
        }

        [Fact]
        public void AMeshIgnoresAHitBeyondTheSegment()
        {
            TriangleMeshSurface wall = Wall(50f);

            Assert.False(wall.GetLineIntersection(
                new Vec3(0f, 5f, 200f), new Vec3(40f, 5f, 200f), out _, out _, true, null));
        }

        [Fact]
        public void TheFloorQueryTakesTheHighestSurfaceAtOrBelowThePoint()
        {
            var deck = new List<Vec3>();
            var idx = new List<int>();
            foreach (float y in new[] { 10f, 20f })
            {
                int b = deck.Count;
                deck.Add(new Vec3(0f, y, 0f));
                deck.Add(new Vec3(40f, y, 0f));
                deck.Add(new Vec3(40f, y, 40f));
                deck.Add(new Vec3(0f, y, 40f));
                idx.AddRange(new[] { b, b + 1, b + 2, b, b + 2, b + 3 });
            }

            var mesh = new TriangleMeshSurface(deck.ToArray(), idx.ToArray());
            mesh.CalculateClosestPoint(new Vec3(20f, 11f, 20f), out Vec3 c, out _, null);

            Assert.Equal(10f, c.Y, 3);
        }

        [Fact]
        public void TheFloorQueryDoesNotLiftABodyStandingBesideAWall()
        {
            TriangleMeshSurface wall = Wall(10f);
            var p = new Vec3(9.9f, 5f, 200f);

            wall.CalculateClosestPoint(p, out Vec3 c, out _, null);

            Assert.Equal(TilemapSurface.NoClosestPoint, c.Y, 3);
        }

        [Fact]
        public void TheTerrainIsKeptWhereTheCellsHaveNoFloor()
        {
            TilemapSurface terrain = TerrainWithCells(out CellSurface cells);
            cells.SetSurfaceForCell(0, Slab(0f, 0f, 40f, 8f));

            terrain.CalculateClosestPoint(new Vec3(600f, 50f, 600f), out Vec3 c, out _, null);

            Assert.Equal(0f, c.Y, 3);
        }

        [Fact]
        public void AStatelFloorWinsOverTheTerrainBelowIt()
        {
            TilemapSurface terrain = TerrainWithCells(out CellSurface cells);
            int cellId = cells.GetCellIdFromPos(new Vec3(100f, 0f, 100f));
            cells.SetSurfaceForCell(cellId, Slab(80f, 80f, 40f, 8f));

            terrain.CalculateClosestPoint(new Vec3(100f, 50f, 100f), out Vec3 c, out _, null);

            Assert.Equal(8f, c.Y, 3);
        }

        sealed class Flat : ITileHeightSource
        {
            public int Width => 256;
            public int Height => 256;
            public float TileSize => 4f;
            public float SampleHeight(int x, int z) => 0f;
        }

        static CharVehicleSim Walker(ISurface surface, Vec3 at)
        {
            var sim = new CharVehicleSim
            {
                Mass = 50f, MaxForce = 10f, MaxVel = 1f, NearProbeOffset = 0.5f,
                SlowingDistance = 1.5f, MovementState = 3, RunSpeedStat = 275f,
                Surface = surface, Position = at,
            };
            sim.EnableFalling();
            sim.DisableSurfaceHug();
            sim.UseSurfaceNormal();
            sim.UpdateMotionConstraints();
            return sim;
        }

        static TilemapSurface TerrainWithCells(out CellSurface cells)
        {
            var terrain = new TilemapSurface(new Flat());
            cells = new CellSurface(25, 25, 1024f, 1024f);
            terrain.Child = cells;
            return terrain;
        }

        [Fact]
        public void ACharacterStandsOnAStatelDeckInsteadOfFallingThroughIt()
        {
            TilemapSurface terrain = TerrainWithCells(out CellSurface cells);
            int cellId = cells.GetCellIdFromPos(new Vec3(100f, 0f, 100f));
            cells.SetSurfaceForCell(cellId, Slab(80f, 80f, 40f, 8f));

            CharVehicleSim sim = Walker(terrain, new Vec3(100f, 12f, 100f));
            for (int i = 0; i < 240; i++)
                sim.Run(1f / 60f);

            Assert.InRange(sim.Position.Y, 7.9f, 8.3f);
            Assert.False(sim.Airborne);
        }

        [Fact]
        public void WithoutTheCellSurfaceTheSameCharacterFallsToTheTerrain()
        {
            var terrain = new TilemapSurface(new Flat());

            CharVehicleSim sim = Walker(terrain, new Vec3(100f, 12f, 100f));
            for (int i = 0; i < 240; i++)
                sim.Run(1f / 60f);

            Assert.InRange(sim.Position.Y, -0.1f, 0.2f);
        }

        [Fact]
        public void ACharacterCannotWalkThroughAStatelWall()
        {
            TilemapSurface terrain = TerrainWithCells(out CellSurface cells);

            TriangleMeshSurface wall = Wall(110f);
            for (int cz = 0; cz < 25; cz++)
                cells.SetSurfaceForCell(cells.GetCellIdFromPos(new Vec3(110f, 0f, cz * 40.96f + 20f)), wall);

            CharVehicleSim sim = Walker(terrain, new Vec3(100f, 0.01f, 100f));
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(System.Math.PI / 2.0));
            sim.SetDirection(1);
            sim.SetForwardDrive(1f);

            for (int i = 0; i < 300; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Position.X < 110f, $"walked through the wall to x={sim.Position.X}");
            Assert.True(sim.Position.X > 104f, $"stopped far too early at x={sim.Position.X}");
        }

        [Fact]
        public void WithoutTheCellSurfaceTheSameCharacterWalksStraightThrough()
        {
            var terrain = new TilemapSurface(new Flat());

            CharVehicleSim sim = Walker(terrain, new Vec3(100f, 0.01f, 100f));
            sim.BodyRotation = Quat.FromAxisAngle(Vec3.ReferenceUp, (float)(System.Math.PI / 2.0));
            sim.SetDirection(1);
            sim.SetForwardDrive(1f);

            for (int i = 0; i < 300; i++)
                sim.Run(1f / 60f);

            Assert.True(sim.Position.X > 110f, $"expected to pass x=110, reached {sim.Position.X}");
        }
    }
}

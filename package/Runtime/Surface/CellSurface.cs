using System.Collections.Generic;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// A flat grid of cells, each holding the surfaces whose geometry
    /// touches it. This is how non-terrain collision is reached from the single surface a vehicle
    /// owns.
    ///
    /// <para>
    /// Geometry is <b>duplicated into every cell it touches</b> — a record's XZ span is 51 m median
    /// against a 40 m cell — so the per-cell hit test rejects any hit outside the cell's own bounds.
    /// That is what makes exactly one cell report each hit, and it is why the ray walk can stop at the
    /// first cell that yields one and still have the nearest.
    /// </para>
    /// </summary>
    public sealed class CellSurface : ISurface
    {
        readonly List<ISurface>[] _cells;

        /// <summary>
        /// The tilemap builds this as <c>(tileWidth / 10, tileHeight / 10, worldWidth, worldDepth)</c>
        /// — see <see cref="TilesPerCell"/>.
        /// </summary>
        public CellSurface(int cellsX, int cellsZ, float worldWidth, float worldDepth)
        {
            CellsX = cellsX;
            CellsZ = cellsZ;
            WorldWidth = worldWidth;
            WorldDepth = worldDepth;
            CellSizeX = cellsX > 0 ? worldWidth / cellsX : 0f;
            CellSizeZ = cellsZ > 0 ? worldDepth / cellsZ : 0f;
            _cells = new List<ISurface>[cellsX > 0 && cellsZ > 0 ? cellsX * cellsZ : 0];
        }

        /// <summary>
        /// Tiles per collision cell: <b>10</b>. It is the tightest divisor that keeps every observed
        /// cell index inside the grid across all 328 outdoor playfields, and the only one that makes
        /// the cells exactly square — which the grid walk requires, because
        /// <see cref="GridSpace.IterateCellsAlongLine"/> uses one cell size for both axes. For a
        /// scale-4 map that is a 40 m cell.
        /// </summary>
        public const int TilesPerCell = 10;

        public int CellsX { get; }
        public int CellsZ { get; }
        public float WorldWidth { get; }
        public float WorldDepth { get; }
        public float CellSizeX { get; }

        /// <summary>
        /// Only <see cref="GetCellIdFromPos"/> uses it — the ray walk and the per-cell bounds test
        /// both use <see cref="CellSizeX"/> on both axes. They agree only because the grid is square.
        /// </summary>
        public float CellSizeZ { get; }

        public int CellCount => _cells.Length;

        /// <summary>The cell holding <paramref name="p"/>, or -1 when it is outside the grid.</summary>
        public int GetCellIdFromPos(Vec3 p)
        {
            if (CellSizeX <= 0f || CellSizeZ <= 0f)
                return -1;

            int cx = (int)(p.X / CellSizeX);                      // truncates toward zero
            int cz = (int)(p.Z / CellSizeZ);

            if (cx < 0 || cx >= CellsX || cz < 0 || cz >= CellsZ)
                return -1;

            return CellsX * cz + cx;
        }

        /// <summary>The cell's surfaces; null when the cell is empty.</summary>
        public List<ISurface> GetSurfaceForCell(int cellId)
            => cellId >= 0 && cellId < _cells.Length ? _cells[cellId] : null;

        public List<ISurface> GetSurfaceForPos(Vec3 p) => GetSurfaceForCell(GetCellIdFromPos(p));

        public void SetSurfaceForCell(int cellId, ISurface surface)
        {
            if (surface == null || cellId < 0 || cellId >= _cells.Length)
                return;

            (_cells[cellId] ??= new List<ISurface>()).Add(surface);
        }

        public void RemoveSurfaceForCell(int cellId, ISurface surface)
            => GetSurfaceForCell(cellId)?.Remove(surface);

        // ---- line query -----------------------------------------------------

        /// <summary>
        /// Walks the cells the line crosses and lets <see cref="Worker"/> test each cell's surfaces.
        /// </summary>
        public bool GetLineIntersection(
            Vec3 start, Vec3 end, out Vec3 hit, out Vec3 normal, bool clipToBounds, object locality)
        {
            var worker = new Worker(this, start, end);
            GridSpace.IterateCellsAlongLine(start, end, ref worker, CellsX, CellsZ, WorldWidth, WorldDepth);

            hit = worker.Hit;
            normal = worker.Normal;
            return worker.HitFound;
        }

        /// <summary>
        /// The per-cell visitor. Keeps the nearest hit within a cell and, because
        /// <see cref="HitFound"/> is never cleared, looks at no later cell once one has been found.
        ///
        /// <para>
        /// A struct, walked by reference: a class here was one allocation per ray, and every body
        /// fires at least six rays a frame.
        /// </para>
        /// </summary>
        struct Worker : ICellWorker
        {
            readonly CellSurface _self;
            readonly Vec3 _a;
            readonly Vec3 _b;
            float _best;

            public Vec3 Hit;
            public Vec3 Normal;
            public bool HitFound;

            public Worker(CellSurface self, Vec3 a, Vec3 b)
            {
                _self = self;
                _a = a;
                _b = b;
                _best = 0f;
                Hit = Vec3.Zero;
                Normal = Vec3.Zero;
                HitFound = false;
            }

            public void DoCell(int cellId)
            {
                if (HitFound)                                     // stop at the first cell
                    return;
                if (cellId == -1)
                    return;

                List<ISurface> surfaces = _self.GetSurfaceForCell(cellId);
                if (surfaces == null)
                    return;

                // The cell's own XZ bounds, with CellSizeX on BOTH axes.
                float size = _self.CellSizeX;
                int cx = cellId % _self.CellsX;
                int cz = cellId / _self.CellsX;
                float loX = cx * size, hiX = (cx + 1) * size;
                float loZ = cz * size, hiZ = (cz + 1) * size;

                for (int i = 0; i < surfaces.Count; i++)
                {
                    ISurface s = surfaces[i];
                    if (s == null)
                        continue;

                    // clip = false, locality = null
                    if (!s.GetLineIntersection(_a, _b, out Vec3 h, out Vec3 n, false, null))
                        continue;

                    // A hit outside this cell belongs to a cell further along the line.
                    if (loX > h.X || hiX < h.X || loZ > h.Z || hiZ < h.Z)
                        continue;

                    float distance = (h - _a).Length;

                    if (!HitFound)
                    {
                        Hit = h;
                        Normal = n;
                        HitFound = true;
                        _best = distance;
                        continue;
                    }

                    if (_best > distance)                          // nearer wins
                    {
                        Hit = h;
                        Normal = n;
                        _best = distance;
                    }
                }
            }
        }

        // ---- closest point --------------------------------------------------

        /// <summary>
        /// Deliberately cheap: it takes the cell at <paramref name="point"/> and hands the query to the
        /// <b>first</b> non-null surface there, then returns, without comparing the cell's surfaces
        /// against each other.
        /// </summary>
        public void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality)
        {
            // The sentinel, not the point -- see TilemapSurface.NoClosestPoint.
            closest = new Vec3(point.X, TilemapSurface.NoClosestPoint, point.Z);
            normal = Vec3.ReferenceUp;

            List<ISurface> surfaces = GetSurfaceForPos(point);
            if (surfaces == null)
                return;

            foreach (ISurface s in surfaces)
            {
                if (s == null)
                    continue;

                s.CalculateClosestPoint(point, out closest, out normal, null);
                return;
            }
        }

        /// <summary>Never vetoes.</summary>
        public bool VetoPosition(ref Vec3 position, object locality, Vec3 previous) => false;
    }
}

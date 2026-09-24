using System;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// The per-cell callback of a grid walk.
    /// </summary>
    public interface ICellWorker
    {
        /// <summary>
        /// Called once per cell the line crosses, <b>in order along the line</b>.
        /// </summary>
        void DoCell(int cellId);
    }

    /// <summary>
    /// The 2D cell walk the collision grid is traversed with.
    /// </summary>
    public static class GridSpace
    {
        /// <summary>
        /// The tDelta scale. The same on both axes, so it never changes which axis steps first.
        /// </summary>
        public const float TScale = 100f;

        /// <summary>
        /// An Amanatides–Woo walk over the cell grid, calling <paramref name="worker"/> once per cell
        /// the line crosses, in order.
        ///
        /// <para>
        /// Three details that are easy to get wrong and all matter:
        /// <b><paramref name="worldDepth"/> is never read</b> — one cell size,
        /// <c>worldWidth / cellsX</c>, serves both axes, which is why the grid has to be square; cell
        /// indices come from <b>truncation</b> toward zero, not floor; and a cell outside the grid
        /// <b>ends the walk</b> rather than being skipped.
        /// </para>
        /// </summary>
        public static void IterateCellsAlongLine(
            Vec3 a, Vec3 b, ICellWorker worker, int cellsX, int cellsZ, float worldWidth, float worldDepth)
        {
            if (worker == null)
                return;

            IterateCellsAlongLine(a, b, ref worker, cellsX, cellsZ, worldWidth, worldDepth);
        }

        /// <summary>
        /// The same walk with the worker passed by reference. A struct worker is neither allocated
        /// nor called through the interface, which matters on the per-frame ground probes.
        /// </summary>
        public static void IterateCellsAlongLine<TWorker>(
            Vec3 a, Vec3 b, ref TWorker worker, int cellsX, int cellsZ, float worldWidth, float worldDepth)
            where TWorker : ICellWorker
        {
            if (cellsX <= 0 || cellsZ <= 0)
                return;

            float cellSize = worldWidth / cellsX;               // worldDepth is unused
            if (cellSize <= 0f)
                return;

            float ax = a.X / cellSize, az = a.Z / cellSize;
            float bx = b.X / cellSize, bz = b.Z / cellSize;

            int cellX = (int)ax, cellZ = (int)az;               // truncating, not floor
            int endX = (int)bx, endZ = (int)bz;

            Step(ax, bx, out int stepX, out float tDeltaX, out float tMaxX);
            Step(az, bz, out int stepZ, out float tDeltaZ, out float tMaxZ);

            // The budget is the Manhattan cell distance, and the loop runs budget + 1 times.
            int budget = Math.Abs(endX - cellX) + Math.Abs(endZ - cellZ);
            if (budget < 0)
                return;

            int row = cellZ * cellsX;
            int rowStep = stepZ * cellsX;

            do
            {
                // Out of range ends the walk; it does not continue to the next cell.
                if (cellX < 0 || cellZ < 0 || cellX >= cellsX || cellZ >= cellsZ)
                    return;

                worker.DoCell(row + cellX);

                if (tMaxZ <= tMaxX)                              // step the smaller tMax
                {
                    tMaxZ += tDeltaZ;
                    cellZ += stepZ;
                    row += rowStep;
                }
                else
                {
                    tMaxX += tDeltaX;
                    cellX += stepX;
                }
            }
            while (--budget >= 0);
        }

        /// <summary>
        /// One axis of the walk's setup. A non-negative delta steps forward and measures to the next
        /// boundary above; a negative one steps back and measures to the boundary below.
        /// </summary>
        static void Step(float from, float to, out int step, out float tDelta, out float tMax)
        {
            float delta = to - from;

            if (!(0f > delta))
            {
                step = 1;
                tDelta = TScale / delta;                         // +Infinity when delta is 0
                tMax = ((float)Math.Floor(from) + 1f - from) * tDelta;
            }
            else
            {
                step = -1;
                tDelta = -TScale / delta;
                tMax = (from - (float)Math.Floor(from)) * tDelta;
            }
        }
    }
}

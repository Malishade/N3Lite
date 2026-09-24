using System;

namespace N3Lite.Surfaces
{
    /// <summary>
    /// The outdoor heightmap lookup over a tilemap's chunked height samples.
    ///
    /// <para>
    /// Takes AODB's parsed chunk arrays directly, so it has no AODB dependency. AODB's
    /// <c>Tilemap.Heightmap</c> is a list of <c>ushort[side, side]</c> indexed <c>[x, z]</c>, which is
    /// the order the lookup reads, so no transpose is needed.
    /// </para>
    ///
    /// <para>
    /// <b>8-bit maps need no special case.</b> A byte widened to 16 bits by multiplying by 0x100 and
    /// shifted back down by 8 on sampling is just the byte, so the height is <c>rawByte * HeightMod</c>
    /// either way. AODB stores the sample itself, so this class just multiplies.
    /// </para>
    /// </summary>
    public sealed class ChunkedTileHeightSource : ITileHeightSource
    {
        // Each chunk flattened to [x * dim1 + z], with its two dimensions alongside: a ushort[,] index
        // and GetLength are both helper calls on some runtimes, and every tile a ray crosses reads
        // four samples.
        readonly ushort[][] _flat;
        readonly int[] _dim0;
        readonly int[] _dim1;
        readonly int _gridWidth;
        readonly int _shift;
        readonly int _mask;
        readonly int _side;
        readonly float _heightMod;

        public int Width { get; }
        public int Height { get; }
        public float TileSize { get; }

        /// <param name="chunks">AODB <c>Tilemap.Heightmap</c>, row-major with <paramref name="gridWidth"/> as the stride.</param>
        /// <param name="gridWidth">AODB <c>Tilemap.GridWidth</c> — chunks per row.</param>
        /// <param name="chunkSide">AODB <c>Tilemap.ChunkSize</c>. This is the <b>sample</b> side, one more than the tile step, because chunks share their edge samples.</param>
        /// <param name="heightMod">AODB <c>Tilemap.HeightMod</c>.</param>
        /// <param name="width">AODB <c>Tilemap.MapWidth</c>.</param>
        /// <param name="height">AODB <c>Tilemap.MapHeight</c>.</param>
        /// <param name="tileSize">AODB <c>Tilemap.MapScale</c>.</param>
        public ChunkedTileHeightSource(
            ushort[][,] chunks,
            int gridWidth,
            int chunkSide,
            float heightMod,
            int width,
            int height,
            float tileSize)
        {
            if (chunks == null)
                throw new ArgumentNullException(nameof(chunks));
            if (gridWidth <= 0)
                throw new ArgumentOutOfRangeException(nameof(gridWidth));
            if (chunkSide <= 1)
                throw new ArgumentOutOfRangeException(nameof(chunkSide));

            int step = chunkSide - 1;
            if ((step & (step - 1)) != 0)
                throw new ArgumentException(
                    $"chunk side {chunkSide} implies a step of {step}, which is not a power of two; " +
                    "the client's chunk lookup shifts and masks, so it cannot address this.",
                    nameof(chunkSide));

            _flat = new ushort[chunks.Length][];
            _dim0 = new int[chunks.Length];
            _dim1 = new int[chunks.Length];
            for (int i = 0; i < chunks.Length; i++)
            {
                ushort[,] chunk = chunks[i];
                if (chunk == null)
                    continue;

                int d0 = chunk.GetLength(0), d1 = chunk.GetLength(1);
                var flat = new ushort[d0 * d1];
                for (int x = 0; x < d0; x++)
                    for (int z = 0; z < d1; z++)
                        flat[x * d1 + z] = chunk[x, z];

                _flat[i] = flat;
                _dim0[i] = d0;
                _dim1[i] = d1;
            }

            _gridWidth = gridWidth;
            _side = chunkSide;
            _mask = step - 1;
            _heightMod = heightMod;

            _shift = 0;
            while ((1 << _shift) < step)
                _shift++;

            Width = width;
            Height = height;
            TileSize = tileSize;
        }

        /// <summary>
        /// <code>
        /// chunk  = patches[(z >> shift) * gridWidth + (x >> shift)]
        /// sample = chunk[x &amp; mask, z &amp; mask]
        /// height = sample * HeightMod
        /// </code>
        /// Out-of-range lookups return 0 rather than throwing: the DDA in
        /// <see cref="TilemapSurface"/> range-checks tiles before calling, but a tile on the far edge
        /// legitimately reads the sample one past it.
        /// </summary>
        public float SampleHeight(int x, int z)
        {
            if (x < 0 || z < 0)
                return 0f;

            int cx = x >> _shift;
            int cz = z >> _shift;
            if (cx >= _gridWidth)
                return 0f;

            int index = cz * _gridWidth + cx;
            if (index < 0 || index >= _flat.Length)
                return 0f;

            ushort[] chunk = _flat[index];
            if (chunk == null)
                return 0f;

            int lx = x & _mask;
            int lz = z & _mask;
            int d1 = _dim1[index];
            if (lx >= _dim0[index] || lz >= d1)
                return 0f;

            return chunk[lx * d1 + lz] * _heightMod;
        }

        /// <summary>The sample side of a chunk, for tests and diagnostics.</summary>
        public int ChunkSide => _side;
    }
}

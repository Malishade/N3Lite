namespace N3Lite.Surfaces
{
    /// <summary>
    /// The terrain data <see cref="TilemapSurface"/> samples. <see cref="ChunkedTileHeightSource"/> is the real implementation; tests supply
    /// synthetic terrain through the same interface.
    /// </summary>
    public interface ITileHeightSource
    {
        /// <summary>Map width in tiles.</summary>
        int Width { get; }

        /// <summary>Map depth in tiles.</summary>
        int Height { get; }

        /// <summary>One tile's world size (the map scale).</summary>
        float TileSize { get; }

        /// <summary>
        /// The height at a sample position, already scaled: <c>sample * HeightMod</c>. Callers pass
        /// sample coordinates, not tile coordinates; the two differ at the far edge because patches
        /// share their edge samples.
        /// </summary>
        float SampleHeight(int x, int z);
    }
}

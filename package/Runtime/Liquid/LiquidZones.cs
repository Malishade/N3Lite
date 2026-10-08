using System;
using System.Collections.Generic;
using N3Lite.Surfaces;

namespace N3Lite.Liquids
{
    /// <summary>
    /// A playfield's liquid, filed by zone: the query that says how high the liquid surface is above a
    /// point.
    ///
    /// <para>
    /// Every collision triangle is filed with each zone a coarse sweep over its bounds touches
    /// (<see cref="AddWater"/>). A zone remembers the highest corner of anything filed with it and the
    /// playfield the highest of those (<see cref="CalculateWaterHeightMax"/>), and the query uses both
    /// to answer quickly when the point is above all the liquid. Within a zone the triangles are tried
    /// in the order they were filed and the <b>first</b> one over the point decides.
    /// </para>
    /// </summary>
    public sealed class LiquidZones
    {
        /// <summary>What the query gives when the point is above every zone's liquid.</summary>
        public const float AbovePlayfield = -9999f;

        /// <summary>What the query gives when its zone has no liquid over the point.</summary>
        public const float NoLiquid = -100000f;

        /// <summary>A zone's height before anything is filed with it.</summary>
        const float EmptyZoneHeight = -1f;

        /// <summary>The margin the sweep's corners are kept inside the map by.</summary>
        const double MapMargin = 0.10000000149011612;

        sealed class Zone
        {
            public readonly List<LiquidTriangle> Triangles = new List<LiquidTriangle>();
            public float MaxHeight = EmptyZoneHeight;
        }

        readonly Dictionary<int, Zone> _zones = new Dictionary<int, Zone>();
        readonly Func<Vec3, int> _zoneOf;
        readonly ISurface _surface;

        /// <summary>The highest zone height above 0; <see cref="NoLiquid"/> until worked out.</summary>
        public float MaxHeight { get; private set; } = NoLiquid;

        /// <summary>
        /// The last test's height, normal and type, as the one shared record the client keeps: the
        /// height is <see cref="NoLiquid"/> after a triangle missed and 0 after a query found nothing.
        /// </summary>
        public float PreviousHeight { get; private set; }
        public Vec3 PreviousNormal { get; private set; }
        public int PreviousType { get; private set; }

        /// <param name="zoneOf">The zone a point is in, as the playfield numbers them.</param>
        /// <param name="surface">
        /// The playfield's surface; its veto is put to the point before the zone is looked up. May be null.
        /// </param>
        public LiquidZones(Func<Vec3, int> zoneOf, ISurface surface = null)
        {
            _zoneOf = zoneOf ?? throw new ArgumentNullException(nameof(zoneOf));
            _surface = surface;
        }

        /// <summary>File a triangle with a zone, raising the zone's height to its highest corner.</summary>
        public void SetWater(int zone, LiquidTriangle triangle)
        {
            if (triangle == null)
                return;
            Zone z = ZoneAt(zone);
            z.Triangles.Add(triangle);
            if (z.MaxHeight < triangle.MaxY)
                z.MaxHeight = triangle.MaxY;
        }

        /// <summary>
        /// File every triangle with every zone a sweep over its bounds touches. The sweep runs in steps
        /// of one zone (<paramref name="tileSize"/> x <paramref name="zoneSize"/>) from one step before
        /// the triangle's corners to one step past them, with the corners and both ends of the sweep
        /// kept inside the map.
        /// </summary>
        /// <param name="tileSize">The map's tile size.</param>
        /// <param name="mapWidth">The map's width in tiles (X).</param>
        /// <param name="mapHeight">The map's height in tiles (Z).</param>
        /// <param name="zoneSize">A zone's width in tiles.</param>
        public void AddWater(IReadOnlyList<LiquidTriangle> triangles, float tileSize, int mapWidth, int mapHeight, int zoneSize)
        {
            if (triangles == null)
                return;

            float step = tileSize * zoneSize;
            // A zone of no size would never step the sweep forward.
            if (!(step > 0f))
                return;
            float maxX = (float)(mapWidth * (double)tileSize - MapMargin);
            float maxZ = (float)(mapHeight * (double)tileSize - MapMargin);

            for (int t = 0; t < triangles.Count; t++)
            {
                LiquidTriangle triangle = triangles[t];
                if (triangle == null)
                    continue;

                Vec3 a = Clamp(triangle.V0, maxX, maxZ);
                Vec3 b = Clamp(triangle.V1, maxX, maxZ);
                Vec3 c = Clamp(triangle.V2, maxX, maxZ);

                float minX = a.X, highX = a.X, minZ = a.Z, highZ = a.Z;
                Widen(b.X, ref minX, ref highX);
                Widen(b.Z, ref minZ, ref highZ);
                Widen(c.X, ref minX, ref highX);
                Widen(c.Z, ref minZ, ref highZ);

                Vec3 low = Clamp(new Vec3(minX - step, 0f, minZ - step), maxX, maxZ);
                Vec3 high = Clamp(new Vec3(step + highX, 0f, step + highZ), maxX, maxZ);

                for (float x = low.X; x < high.X; x += step)
                {
                    for (float z = low.Z; z < high.Z; z += step)
                        SetWater(_zoneOf(new Vec3(x, 0f, z)), triangle);
                }
            }
        }

        /// <summary>The playfield's height: the highest zone height that is above 0.</summary>
        public void CalculateWaterHeightMax()
        {
            foreach (Zone z in _zones.Values)
            {
                if (0f < z.MaxHeight && MaxHeight < z.MaxHeight)
                    MaxHeight = z.MaxHeight;
            }
        }

        /// <summary>
        /// The height of the liquid surface over <paramref name="point"/>, with its normal and liquid
        /// type; <see cref="NoLiquid"/> when there is none, <see cref="AbovePlayfield"/> when the point
        /// is above all of the playfield's liquid.
        ///
        /// <para>
        /// Only a surface whose bottom (<see cref="LiquidTriangle.Depth"/> below it) is at or under
        /// <paramref name="limit"/> counts; -1 means the point's own height. The first triangle over the
        /// point decides: if its bottom is too high, there is no liquid, whatever comes after it.
        /// </para>
        /// </summary>
        /// <param name="point">The point; the surface's veto may move it.</param>
        /// <param name="zone">The point's zone, or -1 to look it up.</param>
        public float GetLiquidSurfaceHeight(ref Vec3 point, out Vec3 normal, out int type, float limit = -1f, int zone = -1)
        {
            normal = Vec3.Zero;
            type = 0;

            if (!(point.Y <= MaxHeight))
            {
                PreviousHeight = 0f;
                return AbovePlayfield;
            }

            if (zone == -1)
            {
                _surface?.VetoPosition(ref point, null, Vec3.Zero);
                zone = _zoneOf(point);
            }

            if (limit == -1f)
                limit = point.Y;

            if (_zones.TryGetValue(zone, out Zone z) && point.Y < z.MaxHeight && z.Triangles.Count != 0)
            {
                List<LiquidTriangle> list = z.Triangles;
                for (int i = 0; i < list.Count; i++)
                {
                    LiquidTriangle t = list[i];
                    float height = t.SurfaceHeight(point);
                    if (!(0f < height))
                    {
                        PreviousHeight = NoLiquid;
                        continue;
                    }

                    normal = t.Normal;
                    type = t.Type;
                    PreviousHeight = height;
                    PreviousNormal = t.Normal;
                    PreviousType = t.Type;
                    if (height - t.Depth <= limit)
                        return height;
                    break;
                }
            }

            PreviousHeight = 0f;
            return NoLiquid;
        }

        /// <summary>The number of triangles filed with a zone (for tests and tools).</summary>
        public int CountIn(int zone) => _zones.TryGetValue(zone, out Zone z) ? z.Triangles.Count : 0;

        /// <summary>A zone's height: its highest filed corner, or -1 with nothing filed.</summary>
        public float HeightOf(int zone) => _zones.TryGetValue(zone, out Zone z) ? z.MaxHeight : EmptyZoneHeight;

        Zone ZoneAt(int zone)
        {
            if (!_zones.TryGetValue(zone, out Zone z))
            {
                z = new Zone();
                _zones.Add(zone, z);
            }
            return z;
        }

        /// <summary>X and Z kept between 0.1 and the map's far edge less 0.1; Y untouched.</summary>
        static Vec3 Clamp(Vec3 v, float maxX, float maxZ)
            => new Vec3(ClampAxis(v.X, maxX), v.Y, ClampAxis(v.Z, maxZ));

        static float ClampAxis(float value, float max)
        {
            const float min = 0.1f;
            if (!(min < value))
                return min;
            if (!(max > value))
                return max;
            return value;
        }

        static void Widen(float value, ref float min, ref float max)
        {
            if (value < min)
                min = value;
            else if (max < value)
                max = value;
        }
    }
}

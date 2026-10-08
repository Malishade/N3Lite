namespace N3Lite.Liquids
{
    /// <summary>
    /// One liquid surface triangle as the collision test sees it: the triangle with its normal facing
    /// up, the plane through it, its bounds and the three edges in the XZ plane.
    ///
    /// <para>
    /// Built only by <see cref="LiquidBuilder"/>; a triangle that reaches here has three distinct
    /// corners and a non-zero normal with Y &gt;= 0.
    /// </para>
    /// </summary>
    public sealed class LiquidTriangle
    {
        public readonly Vec3 V0;
        public readonly Vec3 V1;
        public readonly Vec3 V2;

        /// <summary>Unit length, Y never negative: the corners are swapped to make it so.</summary>
        public readonly Vec3 Normal;

        /// <summary>The liquid type, the water flags' low five bits. Always even here.</summary>
        public readonly int Type;

        /// <summary>How deep the liquid is under its surface (<see cref="LiquidBuilder.DepthOf"/>).</summary>
        public readonly float Depth;

        public readonly float MinX;
        public readonly float MinZ;
        public readonly float MaxX;
        public readonly float MaxY;
        public readonly float MaxZ;

        /// <summary>The plane's constant: <c>-(n · V0)</c>.</summary>
        readonly float _d;

        // The edges V0→V1, V1→V2 and V2→V0 in X and Z.
        readonly float _e0x, _e0z, _e1x, _e1z, _e2x, _e2z;

        internal LiquidTriangle(int type, float depth, Vec3 v0, Vec3 v1, Vec3 v2, Vec3 normal)
        {
            Type = type;
            Depth = depth;
            V0 = v0;
            V1 = v1;
            V2 = v2;
            Normal = normal;

            MinX = Min(v0.X, v1.X, v2.X);
            MinZ = Min(v0.Z, v1.Z, v2.Z);
            MaxX = Max(v0.X, v1.X, v2.X);
            MaxY = Max(v0.Y, v1.Y, v2.Y);
            MaxZ = Max(v0.Z, v1.Z, v2.Z);

            _d = -(normal.Z * v0.Z + normal.X * v0.X + normal.Y * v0.Y);
            _e0x = v1.X - v0.X;
            _e0z = v1.Z - v0.Z;
            _e1x = v2.X - v1.X;
            _e1z = v2.Z - v1.Z;
            _e2x = v0.X - v2.X;
            _e2z = v0.Z - v2.Z;
        }

        /// <summary>
        /// The surface height above <paramref name="point"/>, or 0 when the point is not under this
        /// triangle.
        ///
        /// <para>
        /// The point has to be below the triangle's highest corner and strictly inside its XZ bounds
        /// and its three edges; the height is where the vertical through the point meets the plane, and
        /// only a height above 0 counts.
        /// </para>
        /// </summary>
        public float SurfaceHeight(Vec3 point)
        {
            if (MaxY <= point.Y
                || point.X <= MinX || MaxX <= point.X
                || point.Z <= MinZ || MaxZ <= point.Z)
                return 0f;

            if (0f <= (point.Z - V0.Z) * _e0x - (point.X - V0.X) * _e0z
                || 0f <= (point.Z - V1.Z) * _e1x - (point.X - V1.X) * _e1z
                || 0f <= (point.Z - V2.Z) * _e2x - (point.X - V2.X) * _e2z)
                return 0f;

            float height = (-(point.X * Normal.X) - Normal.Z * point.Z - _d) / Normal.Y;
            return height <= 0f ? 0f : height;
        }

        static float Min(float a, float b, float c)
        {
            float m = b <= a ? b : a;
            return c <= m ? c : m;
        }

        static float Max(float a, float b, float c)
        {
            float m = a <= b ? b : a;
            return m < c ? c : m;
        }
    }
}

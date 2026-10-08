namespace N3Lite.Liquids
{
    /// <summary>
    /// What the ground query found of the liquid at a body's ground point: the surface height, its
    /// normal and the liquid type. A surface fills it when it is given one; the body reads it to swim.
    /// </summary>
    public sealed class LiquidMedium
    {
        /// <summary>The surface height, or one of <see cref="LiquidZones"/>' "no liquid" answers.</summary>
        public float Height;
        public Vec3 Normal;
        public int Type;
    }

    /// <summary>
    /// The ground query's liquid rule: a ground point more than <see cref="Depth"/> under the liquid
    /// surface is raised to that depth and takes the liquid's normal, which is what holds a body up at
    /// swimming height.
    /// </summary>
    public static class LiquidLift
    {
        /// <summary>How far under the surface a body is held (a float 1.2 used as a double).</summary>
        public const double Depth = 1.2000000476837158;

        /// <summary>Raise <paramref name="closest"/> to the swimming depth under <paramref name="liquidHeight"/>.</summary>
        public static void Apply(float liquidHeight, Vec3 liquidNormal, ref Vec3 closest, ref Vec3 normal)
        {
            if (closest.Y < liquidHeight - Depth)
            {
                normal = liquidNormal;
                closest.Y = (float)(liquidHeight - Depth);
            }
        }
    }
}

namespace N3Lite.Surfaces
{
    /// <summary>
    /// A surface whose closest-point query also reports the liquid at the ground point into the body's
    /// <see cref="Liquids.LiquidMedium"/> (stock's <c>LiquidMediumData_t*</c> argument).
    /// </summary>
    public interface ILiquidSurface
    {
        void CalculateClosestPoint(Vec3 point, out Vec3 closest, out Vec3 normal, object locality, Liquids.LiquidMedium medium);
    }
}

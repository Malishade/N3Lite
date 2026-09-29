using System.Collections.Generic;

namespace N3Lite
{
    /// <summary>
    /// The waypoint list an <see cref="NpcVehicleSim"/> follows.
    ///
    /// <para>
    /// It keeps two parallel lists: the waypoints themselves and, per segment, a <b>unit direction and
    /// its length</b>. Both are built incrementally by <see cref="AddWaypoint"/>, so there is no
    /// rebuild step.
    /// </para>
    /// </summary>
    public sealed class Path
    {
        readonly List<Vec3> _waypoints = new List<Vec3>();
        readonly List<Vec3> _directions = new List<Vec3>();   // unit, one per segment
        readonly List<float> _lengths = new List<float>();

        /// <summary>Number of waypoints.</summary>
        public int Size => _waypoints.Count;

        public bool Empty => _waypoints.Count == 0;

        /// <summary>Sum of the segment lengths, accumulated by <see cref="AddWaypoint"/>.</summary>
        public float TotalLength { get; private set; }

        /// <summary>Stored, but unused by the steering.</summary>
        public float Radius { get; set; }

        public Vec3 GetWaypoint(int index) => _waypoints[index];

        /// <summary>The segment's unit direction.</summary>
        public Vec3 GetDir(int segment) => _directions[segment];

        public float GetSegLen(int segment) => _lengths[segment];

        public void Clear()
        {
            _waypoints.Clear();
            _directions.Clear();
            _lengths.Clear();
            TotalLength = 0f;
        }

        /// <summary>
        /// Appends a waypoint.
        ///
        /// <para>
        /// <b>The Y is discarded</b>: waypoints are stored as <c>(x, 0, z)</c>. Paths are flat, which
        /// is why <see cref="NpcVehicleSim"/> forces the steering target back to the body's own height.
        /// </para>
        ///
        /// <para>
        /// A waypoint that repeats the previous one is dropped: the segment direction is built first
        /// and a zero one is not committed.
        /// </para>
        /// </summary>
        public void AddWaypoint(Vec3 waypoint)
        {
            var flat = new Vec3(waypoint.X, 0f, waypoint.Z);

            if (_waypoints.Count == 0)
            {
                _waypoints.Add(flat);
                return;
            }

            Vec3 direction = flat - _waypoints[_waypoints.Count - 1];
            if (direction.IsZero)
                return;

            float length = direction.Length;
            _waypoints.Add(flat);
            _directions.Add(direction * (1f / length));
            _lengths.Add(length);
            TotalLength += length;
        }

        /// <summary>
        /// Walks the segments consuming <paramref name="distance"/> and returns
        /// <c>waypoint[segment] + direction[segment] * remaining</c>. Past the end of the path it
        /// returns the <b>final waypoint</b>; it does not extrapolate.
        /// </summary>
        public Vec3 MapPathDistanceToPoint(float distance)
        {
            if (_waypoints.Count == 0)
                return Vec3.Zero;

            float remaining = distance;
            int segments = _waypoints.Count - 1;

            for (int segment = 0; segment < segments; segment++)
            {
                float length = _lengths[segment];

                // `remaining <= length` is the hit, so a zero distance lands on the first waypoint
                // rather than skipping a zero-length segment.
                if (remaining <= length)
                    return _waypoints[segment] + _directions[segment] * remaining;

                remaining -= length;
            }

            return _waypoints[_waypoints.Count - 1];
        }
    }

    /// <summary>
    /// A point that slides along a <see cref="Path"/> at a fixed speed. The NPC steers at the guide,
    /// not at a waypoint, so it cuts corners instead of snapping between legs.
    ///
    /// <para>
    /// The whole model is <c>guidePos = path.MapPathDistanceToPoint(maxSpeed * time)</c>. The position
    /// is <b>cached</b>: it only moves when one of the update calls runs.
    /// </para>
    /// </summary>
    public sealed class PathGuide
    {
        public float MaxSpeed { get; private set; }

        public float Time { get; private set; }

        /// <summary>The guide's point, as of the last update call.</summary>
        public Vec3 GuidePos { get; private set; }

        public Path Path { get; private set; }

        /// <summary>
        /// Does <b>not</b> compute the guide position — only <see cref="RestartGuide"/> and the update
        /// calls do, so a freshly constructed guide reports <c>(0,0,0)</c> until one of them runs.
        /// </summary>
        public PathGuide(Path path, float maxSpeed, float time)
        {
            Path = path;
            MaxSpeed = maxSpeed;
            Time = time;
        }

        /// <summary>Everything zero, no path.</summary>
        public PathGuide()
        {
        }

        /// <summary>
        /// Advance by a delta and re-map.
        ///
        /// <para>
        /// Guarded against a null or empty path, like <see cref="UpdateTime"/> and
        /// <see cref="RestartGuide"/>.
        /// </para>
        /// </summary>
        public void UpdateAddTime(float dt)
        {
            Time += dt;

            if (Path != null && !Path.Empty)
                GuidePos = Path.MapPathDistanceToPoint(Time * MaxSpeed);
        }

        /// <summary>Stores only; does not re-map.</summary>
        public void UpdateMaxSpeed(float maxSpeed) => MaxSpeed = maxSpeed;

        /// <summary>Set the absolute time and re-map.</summary>
        public void UpdateTime(float time)
        {
            Time = time;

            if (Path != null && !Path.Empty)
                GuidePos = Path.MapPathDistanceToPoint(MaxSpeed * time);
        }

        public void RestartGuide(Path path, float maxSpeed, float time)
        {
            Path = path;
            MaxSpeed = maxSpeed;
            Time = time;

            if (path != null && !path.Empty)
                GuidePos = path.MapPathDistanceToPoint(maxSpeed * time);
        }
    }
}

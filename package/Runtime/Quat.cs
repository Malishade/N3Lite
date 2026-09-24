using System;

namespace N3Lite
{
    /// <summary>
    /// A rotation, stored x, y, z, w. Left-handed, like <see cref="Vec3"/>.
    /// </summary>
    public struct Quat : IEquatable<Quat>
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Quat(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static readonly Quat Identity = new Quat(0f, 0f, 0f, 1f);

        /// <summary>Rotation of <paramref name="radians"/> about a <b>unit</b> <paramref name="axis"/>.</summary>
        public static Quat FromAxisAngle(Vec3 axis, float radians)
        {
            float half = radians * 0.5f;
            float s = (float)Math.Sin(half);
            return new Quat(axis.X * s, axis.Y * s, axis.Z * s, (float)Math.Cos(half));
        }

        /// <summary>Hamilton product: <paramref name="a"/> applied after <paramref name="b"/>.</summary>
        public static Quat operator *(Quat a, Quat b) => new Quat(
            a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
            a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
            a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
            a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z);

        /// <summary>Rotate a vector by this quaternion.</summary>
        public static Vec3 operator *(Quat q, Vec3 v)
        {
            var u = new Vec3(q.X, q.Y, q.Z);
            float s = q.W;
            return u * (2f * Vec3.Dot(u, v))
                 + v * (s * s - Vec3.Dot(u, u))
                 + Vec3.Cross(u, v) * (2f * s);
        }

        /// <summary>
        /// The body orientation built from a forward and an up vector, as the orientation update
        /// does it.
        ///
        /// <para>
        /// Two details matter:
        /// </para>
        /// <list type="bullet">
        ///   <item><b>An up vector with <c>Y &lt;= 0</c> is replaced by world up</b>, so a body is
        ///     never turned upside down by a bad normal.</item>
        ///   <item>Forward is projected onto the plane by adjusting <b>only its Y</b>:
        ///     <c>forward.y -= dot(forward, up) / up.y</c>. The result is still perpendicular to
        ///     <c>up</c>, and it is what tilts a walker's forward along a slope — which is how the
        ///     swept solver comes to see an uphill move at all.</item>
        /// </list>
        ///
        /// <para>
        /// The basis-to-quaternion step is stock's own, which takes <c>up</c> as given — see
        /// <see cref="FromBasis"/>.
        /// </para>
        /// </summary>
        public static Quat LookRotation(Vec3 forward, Vec3 up) => LookRotation(ref forward, ref up);

        /// <summary>
        /// <see cref="LookRotation(Vec3, Vec3)"/>, leaving its working in the arguments the way stock
        /// does: <paramref name="up"/> squared to world up when its <c>Y &lt;= 0</c>, and
        /// <paramref name="forward"/> projected and normalised — or, when the projection leaves nothing,
        /// <c>cross((0,0,1), up)</c>. The orientation update caches that forward, which is why the
        /// cached body forward is always a unit vector.
        /// </summary>
        public static Quat LookRotation(ref Vec3 forward, ref Vec3 up)
        {
            if (up.Y <= 0f)
                up = Vec3.ReferenceUp;

            float dot = Vec3.Dot(forward, up);
            forward.Y -= dot / up.Y;

            if (NormaliseInPlace(ref forward) == 0f)
                forward = Vec3.Cross(Vec3.ReferenceForward, up);

            return FromBasis(forward, up);
        }

        /// <summary>
        /// Stock's normalise: scales <paramref name="v"/> to unit length and returns the length it ends
        /// with. Between 1e-8 and 1e-4 it normalises twice; at or below 1e-8 it zeroes the vector and
        /// returns 0.
        /// </summary>
        static float NormaliseInPlace(ref Vec3 v)
        {
            float length = (float)Math.Sqrt(v.LengthSquared);

            if (length > 0.0001)
            {
                v = v * (1f / length);
                return length;
            }

            if (length > 1e-8f)
            {
                v = v * (1f / length);
                length = (float)Math.Sqrt(v.LengthSquared);
                v = v * (1f / length);
                return length;
            }

            v = Vec3.Zero;
            return 0f;
        }

        /// <summary>
        /// Stock's basis-to-quaternion step. It takes the vectors as they are — <c>up</c> is not
        /// normalised and not re-derived, <c>right = cross(up, forward)</c> is not normalised — and
        /// builds the quaternion directly as
        /// <c>(up.z - fwd.y, fwd.x - right.z, right.y - up.x, 1 + right.x + up.y + fwd.z)</c>,
        /// normalised. Only when that is too short (squared length at most 0.01, a turn near half a
        /// revolution) does it fall back to the full matrix conversion, with the rows right, up and
        /// forward.
        /// </summary>
        static Quat FromBasis(Vec3 forward, Vec3 up)
        {
            Vec3 right = Vec3.Cross(up, forward);

            var q = new Quat(
                up.Z - forward.Y,
                forward.X - right.Z,
                right.Y - up.X,
                right.X + 1f + up.Y + forward.Z);

            float lengthSquared = q.X * q.X + q.Y * q.Y + q.Z * q.Z + q.W * q.W;
            if (lengthSquared > 0.01)
            {
                float length = (float)Math.Sqrt(lengthSquared);
                return new Quat(q.X / length, q.Y / length, q.Z / length, q.W / length);
            }

            return FromRows(right, up, forward);
        }

        /// <summary>
        /// The matrix conversion <see cref="FromBasis"/> falls back on. The rows are right, up and
        /// forward; the branch is on the trace, then on the largest diagonal term.
        /// </summary>
        static Quat FromRows(Vec3 right, Vec3 up, Vec3 forward)
        {
            // Stock transposes the matrix before reading it, so mRC is row C, component R.
            float m00 = right.X, m01 = up.X, m02 = forward.X;
            float m10 = right.Y, m11 = up.Y, m12 = forward.Y;
            float m20 = right.Z, m21 = up.Z, m22 = forward.Z;

            float trace = m00 + m11 + m22;
            if (trace >= 0f)
            {
                float s = (float)Math.Sqrt(1f + trace);
                float k = 0.5f / s;
                return new Quat((m21 - m12) * k, (m02 - m20) * k, (m10 - m01) * k, s * 0.5f);
            }

            int i = m11 > m00 ? 1 : 0;
            if ((i == 1 ? m11 : m00) < m22)
                i = 2;

            if (i == 0)
            {
                float s = (float)Math.Sqrt(m00 - (m11 + m22) + 1f);
                float k = 0.5f / s;
                return new Quat(s * 0.5f, (m01 + m10) * k, (m20 + m02) * k, (m21 - m12) * k);
            }

            if (i == 1)
            {
                float s = (float)Math.Sqrt(m11 - (m22 + m00) + 1f);
                float k = 0.5f / s;
                return new Quat((m01 + m10) * k, s * 0.5f, (m12 + m21) * k, (m02 - m20) * k);
            }

            {
                float s = (float)Math.Sqrt(m22 - (m00 + m11) + 1f);
                float k = 0.5f / s;
                return new Quat((m20 + m02) * k, (m12 + m21) * k, s * 0.5f, (m10 - m01) * k);
            }
        }

        /// <summary>The inverse rotation, for a unit quaternion.</summary>
        public Quat Conjugate => new Quat(-X, -Y, -Z, W);

        public float Length => (float)Math.Sqrt(X * X + Y * Y + Z * Z + W * W);

        public Quat Normalized
        {
            get
            {
                float len = Length;
                if (len == 0f)
                    return Identity;
                return new Quat(X / len, Y / len, Z / len, W / len);
            }
        }

        public bool Equals(Quat other) => X == other.X && Y == other.Y && Z == other.Z && W == other.W;

        public override bool Equals(object obj) => obj is Quat q && Equals(q);

        public override int GetHashCode() => (X, Y, Z, W).GetHashCode();

        public override string ToString() => $"({X}, {Y}, {Z}, {W})";
    }
}

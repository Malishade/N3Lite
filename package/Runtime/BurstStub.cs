#if !UNITY_5_3_OR_NEWER
using System;

// Outside Unity there is no Burst package; the attribute only has to exist. The methods it marks then
// run as ordinary C#, exactly as they do inside Unity with Burst switched off.
namespace Unity.Burst
{
    public enum FloatMode { Default, Strict, Deterministic, Fast }

    public enum FloatPrecision { Standard, High, Medium, Low }

    [AttributeUsage(AttributeTargets.Class | AttributeTargets.Struct | AttributeTargets.Method | AttributeTargets.Assembly)]
    public sealed class BurstCompileAttribute : Attribute
    {
        public FloatMode FloatMode { get; set; }
        public FloatPrecision FloatPrecision { get; set; }
        public bool CompileSynchronously { get; set; }
    }
}
#endif

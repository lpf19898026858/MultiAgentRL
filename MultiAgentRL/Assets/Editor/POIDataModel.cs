using UnityEngine;
using System.Collections.Generic;
using YamlDotNet.Serialization; // 确保导入 YamlDotNet

// 为了将Unity的Vector3序列化为YAML中的 {x: y: z:} 格式
// YamlDotNet 默认会将Vector3序列化为 x: ..., y: ..., z: ...
// 我们需要一个自定义的struct来强制它在一行
[System.Serializable] // 可以在Unity Inspector中显示
public struct SerializableVector3
{
    public float x;
    public float y;
    public float z;

    public SerializableVector3(Vector3 v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
    }

    // 隐式转换，方便在Vector3和SerializableVector3之间转换
    public static implicit operator Vector3(SerializableVector3 sv) => new Vector3(sv.x, sv.y, sv.z);
    public static implicit operator SerializableVector3(Vector3 v) => new SerializableVector3(v);
}
// --- 新增：用于命名点的结构 ---
public class NamedPoint
{
    // <<< 关键修正
    [YamlMember(Alias = "name")]
    public string Name { get; set; }

    [YamlMember(Alias = "point")]
    public SerializableVector3 Point { get; set; }
}
// --- 新增：用于简化边界的数据结构 ---
public class SimplifiedBoundary
{
    [YamlMember(Alias = "min")]
    public SerializableVector3 Min { get; set; }

    [YamlMember(Alias = "max")]
    public SerializableVector3 Max { get; set; }
}

// === 新增的 BoundaryGroup 类，用于将每个边界点集合与其名称关联 ===
public class BoundaryGroup
{
    // 这个属性在 YAML 中会变成 `name` (因为 UnderscoredNamingConvention)
    public string Name { get; set; }
    // 这个属性在 YAML 中会变成 `points` (因为 UnderscoredNamingConvention)
    public List<SerializableVector3> Points { get; set; } = new List<SerializableVector3>();
}

// 对应YAML中的每个POI条目
public class POIEntry
{
    [YamlMember(Alias = "name")]
    public string Name { get; set; }

    [YamlMember(Alias = "description")]
    public string Description { get; set; }

    [YamlMember(Alias = "type")]
    public string Type { get; set; }

    [YamlMember(Alias = "position")]
    public SerializableVector3 Position { get; set; }
    
    [YamlMember(Alias = "candidate_interaction_points", ApplyNamingConventions = false, DefaultValuesHandling = DefaultValuesHandling.OmitNull)]
    public List<NamedPoint> CandidateInteractionPoints { get; set; }

    [YamlMember(Alias = "simplified_boundary", ApplyNamingConventions = false, DefaultValuesHandling = DefaultValuesHandling.OmitNull)]
    public SimplifiedBoundary SimplifiedBoundary { get; set; }
}

// 对应整个YAML文件的根结构
public class POIData
{
    // 显式指定 YAML 键名为 "points_of_interest"
    [YamlMember(Alias = "points_of_interest")]
    public List<POIEntry> PointsOfInterest { get; set; } = new List<POIEntry>();
}
// POIInfo.cs
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using YamlDotNet.Serialization; // 确保导入 YamlDotNet

#if UNITY_EDITOR
using UnityEditor;
#endif

public class NamedPoint
{
    // <<< 关键修正
    [YamlMember(Alias = "name")]
    public string Name { get; set; }

    [YamlMember(Alias = "point")]
    public SerializableVector3 Point { get; set; }
}
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
public class POIInfo : MonoBehaviour
{
    [Header("POI 基本信息")]
    public string poiName;
    [TextArea(3, 5)]
    public string description = "A point of interest.";
    public string poiType = "unspecified";
    
    [Header("导航与边界")]
    [Tooltip("如果需要手动指定唯一的交互点（覆盖所有自动计算），请将目标Transform拖到此处。")]
    public Transform manualInteractionPointTarget;
    [Tooltip("无人机可飞行的全局X轴最小/最大值")]
    public Vector2 flyableXBounds = new Vector2(-32f, 40f);
    
    [Tooltip("无人机可飞行的全局Y轴（高度）最小/最大值")]
    public Vector2 flyableYBounds = new Vector2(0.5f, 6f);

    [Tooltip("无人机可飞行的全局Z轴最小/最大值")]
    public Vector2 flyableZBounds = new Vector2(-45f, 40f);

    [Tooltip("勾选此项，将根据所有子碰撞体计算并导出一个简化的边界。")]
    public bool exportSimplifiedBoundary = true;
    
    [Tooltip("勾选此项，将为此POI自动生成或使用手动的候选交互点。")]
    public bool generateCandidatePoints = true;
    

// GetBestInteractionPoint 方法的最终、最完善版本
public NamedPoint GetBestInteractionPoint(float safeDistance = 2.5f, float fixedHeight = 2.0f, float accessibilityCheckRadius = 2.5f, float boundaryMargin = 2.0f)
{
    if (!generateCandidatePoints) return null;

    if (manualInteractionPointTarget != null)
    {
        if (IsPointInSafeFlyableZone(manualInteractionPointTarget.position, boundaryMargin))
        {
            return new NamedPoint { Name = "manual_override_valid", Point = manualInteractionPointTarget.position };
        }
        Debug.LogWarning($"手动指定的点 for {gameObject.name} 不在安全可飞行区域内!", this);
        return null;
    }

    if (GetSimplifiedBounds(out Bounds totalBounds))
    {
        float clampedHeight = Mathf.Clamp(fixedHeight, flyableYBounds.x + boundaryMargin, flyableYBounds.y - boundaryMargin);

        List<(string Name, Vector3 Point)> validCandidates = new List<(string Name, Vector3 Point)>();
        
        // <<< 核心修正：带回迭代逻辑，但使用正确的验证方式 >>>
        float currentSafeDistance = safeDistance;
        int maxAttempts = 5; // 最多尝试5次，每次增加1米
        float distanceIncrement = 1.0f;

        for (int attempt = 0; attempt < maxAttempts; attempt++)
        {
            // 1. 生成基于【当前距离】的候选点列表
            //    注意：这里直接使用物体的原始 totalBounds.center，不做任何预先钳制！
            var rawCandidates = new List<(string Name, Vector3 Point)>
            {
                ("approach_pos_x", new Vector3(totalBounds.max.x + currentSafeDistance, clampedHeight, totalBounds.center.z)),
                ("approach_neg_x", new Vector3(totalBounds.min.x - currentSafeDistance, clampedHeight, totalBounds.center.z)),
                ("approach_pos_z", new Vector3(totalBounds.center.x, clampedHeight, totalBounds.max.z + currentSafeDistance)),
                ("approach_neg_z", new Vector3(totalBounds.center.x, clampedHeight, totalBounds.min.z - currentSafeDistance))
            };

            validCandidates.Clear(); // 清空上一轮可能失败的结果
            foreach (var candidate in rawCandidates)
            {
                // 2. 对【完整的候选点Vector3】进行安全区验证
                //    这是最关键的一步，它同时检查了所有轴
                if (IsPointInSafeFlyableZone(candidate.Point, boundaryMargin))
                {
                    validCandidates.Add(candidate);
                }
            }
            
            // 3. 如果在当前距离下找到了任何一个有效的点，就停止尝试，不再增加距离
            if (validCandidates.Any())
            {
                break;
            }

            // 4. 如果没找到，就增加距离，进入下一轮循环
            currentSafeDistance += distanceIncrement;
        }

        // 循环结束后，如果 validCandidates 列表依然是空的，说明多次尝试都失败了
        if (!validCandidates.Any())
        {
            Debug.LogWarning($"为 {gameObject.name} 尝试了 {maxAttempts} 次后，仍未找到合适的交互点。", this);
            return null;
        }

        // 对找到的有效点进行评分，选出最优的
        var scoredCandidates = validCandidates
            .Select(c => new { PointData = c, Score = CalculateAccessibilityScore(c.Point, accessibilityCheckRadius) })
            .ToList();

        if (scoredCandidates.Any())
        {
            var bestCandidate = scoredCandidates.OrderByDescending(t => t.Score).First();
            return new NamedPoint { Name = "best_" + bestCandidate.PointData.Name, Point = bestCandidate.PointData.Point };
        }
    }
    return null;
}
// <<< 确保您的脚本里有这个辅助方法
private bool IsPointInSafeFlyableZone(Vector3 point, float margin)
{
    // 检查点是否在缩小的边界内部
    return point.x >= flyableXBounds.x + margin && point.x <= flyableXBounds.y - margin &&
           point.y >= flyableYBounds.x + margin && point.y <= flyableYBounds.y - margin &&
           point.z >= flyableZBounds.x + margin && point.z <= flyableZBounds.y - margin;
}

// 别忘了您原始代码里的 IsPointInFlyableZone 也可能需要
private bool IsPointInFlyableZone(Vector3 point)
{
    return point.x >= flyableXBounds.x && point.x <= flyableXBounds.y &&
           point.y >= flyableYBounds.x && point.y <= flyableYBounds.y &&
           point.z >= flyableZBounds.x && point.z <= flyableZBounds.y;
}

    private float CalculateAccessibilityScore(Vector3 point, float radius)
    {
        Collider[] colliders = Physics.OverlapSphere(point, radius);
        int obstacleCount = 0;
        foreach (var col in colliders)
        {
            if (!col.transform.IsChildOf(this.transform) && !col.isTrigger) // 确保不计算自己或触发器
            {
                obstacleCount++;
            }
        }
        return 1.0f / (1.0f + obstacleCount);
    }

    void OnDrawGizmos()
    {
        // 绘制中心点
        Gizmos.color = Color.cyan;
        Gizmos.DrawSphere(transform.position, 0.5f);
        
        #if UNITY_EDITOR
        string displayLabel = string.IsNullOrEmpty(poiName) ? gameObject.name : poiName;
        Handles.Label(transform.position + Vector3.up * 1.5f, displayLabel);
        
        // --- 修正后的Gizmos绘制逻辑 ---
        if (generateCandidatePoints)
        {
            var bestPoint = GetBestInteractionPoint();
            if (bestPoint != null)
            {
                Gizmos.color = Color.green;
                // 注意这里需要从 SerializableVector3 隐式转换回 Vector3
                Gizmos.DrawSphere(bestPoint.Point, 0.5f);
                Gizmos.DrawLine(transform.position, bestPoint.Point);
                Handles.Label(bestPoint.Point + Vector3.up, bestPoint.Name);
            }
        }
        #endif

        // 绘制简化边界
        if (exportSimplifiedBoundary && GetSimplifiedBounds(out Bounds bounds))
        {
            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
            Gizmos.DrawWireCube(bounds.center, bounds.size);
        }
    }

    public bool GetSimplifiedBounds(out Bounds bounds)
    {
        bounds = new Bounds(transform.position, Vector3.zero);
        var colliders = GetComponentsInChildren<Collider>();
        if (colliders.Length == 0) return false;

        bool hasBounds = false;
        foreach(var col in colliders)
        {
            if(!col.isTrigger)
            {
                if(!hasBounds)
                {
                    bounds = col.bounds;
                    hasBounds = true;
                }
                else
                {
                    bounds.Encapsulate(col.bounds);
                }
            }
        }
        return hasBounds;
    }
}
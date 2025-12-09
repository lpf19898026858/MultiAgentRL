using UnityEngine;
using UnityEditor;
using System.IO;
using System.Collections.Generic;
using YamlDotNet.Serialization;

public class POIExporter : EditorWindow
{
    private string outputFileName = "points_of_interest.yaml";
    private bool convertToRosCoordinates = true;

    [MenuItem("Tools/POI/Export POIs to YAML")]
    public static void ShowWindow()
    {
        GetWindow<POIExporter>("POI Exporter");
    }

    void OnGUI()
    {
        GUILayout.Label("导出场景中的兴趣点 (POI)", EditorStyles.boldLabel);

        outputFileName = EditorGUILayout.TextField("输出文件名", outputFileName);

        EditorGUILayout.Space();
        convertToRosCoordinates = EditorGUILayout.Toggle("转换为 ROS 坐标系", convertToRosCoordinates);

        EditorGUILayout.HelpBox("ROS 坐标系约定：\n" +
                                "Unity X (右) -> ROS Y (左) \n" +
                                "Unity Y (上) -> ROS Z (上) \n" +
                                "Unity Z (前) -> ROS X (前) \n" +
                                "因此：ROS (x, y, z) = Unity (z, -x, y)", MessageType.Info);
        
        EditorGUILayout.Space();

        if (GUILayout.Button("开始导出 POIs"))
        {
            ExportPOIs();
        }

        EditorGUILayout.HelpBox("请确保你的POI GameObject上附加了 'POIInfo' 组件，并且你已在 Inspector 中填写了相应的信息。\n" +
                                "对于边界点，你可以选择 'Manual' 手动输入，或选择 'ColliderBounds' 让导出器自动从 GameObject 及其子物体上的每个碰撞器获取其世界轴对齐包围盒的8个角点。", MessageType.Info);
    }

private void ExportPOIs()
{
    string savePath = EditorUtility.SaveFilePanel("保存 POI YAML 文件", Application.dataPath, outputFileName, "yaml");
    if (string.IsNullOrEmpty(savePath)) return;

    POIInfo[] allPoiInfos = FindObjectsOfType<POIInfo>();
    if (allPoiInfos.Length == 0)
    {
        EditorUtility.DisplayDialog("未找到 POIs", "场景中未找到任何附加了 'POIInfo' 组件的 GameObject。", "OK");
        return;
    }

    POIData data = new POIData();
    foreach (POIInfo poiInfo in allPoiInfos)
    {
        var entry = new POIEntry
        {
            Name = string.IsNullOrEmpty(poiInfo.poiName) ? poiInfo.gameObject.name : poiInfo.gameObject.name,
            Description = poiInfo.description,
            Type = poiInfo.poiType,
            Position = ConvertVector(poiInfo.transform.position)
        };

             if (poiInfo.generateCandidatePoints)
            {
                var bestPoint = poiInfo.GetBestInteractionPoint();
                if (bestPoint != null)
                {
                    // 初始化列表
                    entry.CandidateInteractionPoints = new List<NamedPoint>();
                    // 添加最佳点, 注意坐标转换
                    entry.CandidateInteractionPoints.Add(new NamedPoint
                    {
                        Name = bestPoint.Name,
                        Point = ConvertVector(bestPoint.Point) // bestPoint.Point 是 SerializableVector3, 隐式转换为Vector3
                    });
                }
            }

        // --- 处理简化边界 ---
        if (poiInfo.exportSimplifiedBoundary)
        {
            if (poiInfo.GetSimplifiedBounds(out Bounds bounds))
            {
                entry.SimplifiedBoundary = new SimplifiedBoundary
                {
                    Min = ConvertVector(bounds.min),
                    Max = ConvertVector(bounds.max)
                };
            }
        }
        data.PointsOfInterest.Add(entry);
    }
    
    // ... (序列化和保存部分保持不变，包括手动替换key名称) ...
    // ...
        var serializer = new SerializerBuilder()
            .ConfigureDefaultValuesHandling(DefaultValuesHandling.OmitNull)
            .Build();
    
       string yamlContent = serializer.Serialize(data);
        
        File.WriteAllText(savePath, yamlContent);
        EditorUtility.DisplayDialog("导出成功", $"成功导出 {data.PointsOfInterest.Count} 个 POI 到:\n{savePath}", "OK");
}

     private SerializableVector3 ConvertVector(Vector3 vector)
    {
        if (convertToRosCoordinates)
        {
            return new SerializableVector3(new Vector3(vector.z, -vector.x, vector.y));
        }
        return new SerializableVector3(vector); // 总是返回 SerializableVector3
    }
}
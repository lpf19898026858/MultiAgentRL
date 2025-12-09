using UnityEngine;
using System.Collections.Generic;

public class DroneTrajectoryVisualizer : MonoBehaviour
{
    [Header("Drone Object Reference")]
    public GameObject droneGameObject; // 将你的无人机模型GameObject拖拽到这里

    [Header("Line Visuals")]
    public LineRenderer lineRenderer;
    public float maxLineLifetime = 10.0f; // 轨迹线的最大生命周期（秒），超过此时间会逐渐消失
    public float lineWidth = 0.1f; // 轨迹线的宽度
    public Color lineColor = Color.blue; // 轨迹线的颜色

    [Header("Trajectory Point Management")]
    public float minDistanceThreshold = 0.05f; // 只有当无人机移动超过此距离时才添加新的轨迹点
    private Vector3 lastAddedPosition = Vector3.negativeInfinity; // 用于判断是否添加新点

    // 存储轨迹点及其时间戳
    private List<Vector3> trajectoryPoints;
    private List<float> pointTimestamps;

    void Awake()
    {
        // 确保 LineRenderer 组件存在
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.GetComponent<LineRenderer>();
            if (lineRenderer == null)
            {
                lineRenderer = gameObject.AddComponent<LineRenderer>();
            }
        }

        // 初始化列表
        trajectoryPoints = new List<Vector3>();
        pointTimestamps = new List<float>();

        // 配置 LineRenderer 的基础属性
        lineRenderer.startWidth = lineWidth;
        lineRenderer.endWidth = lineWidth;
        lineRenderer.useWorldSpace = true; // 在世界坐标系中绘制

        // 设置一个默认材质，否则线可能不显示
        // 你可以创建一个自己的材质 (Shader: Unlit/Color 或 Sprites/Default)
        if (lineRenderer.material == null)
        {
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        }
        lineRenderer.startColor = lineColor;
        lineRenderer.endColor = lineColor; // 初始时，起止颜色相同

        // 配置 LineRenderer 的颜色渐变，用于实现尾部逐渐消失的效果
        SetLineGradient();
    }

    void Start()
    {
        if (droneGameObject == null)
        {
            Debug.LogError("Drone GameObject is not assigned! Trajectory visualization will not work.", this);
            enabled = false; // 禁用此脚本
            return;
        }

        // 第一次添加当前无人机位置，确保线从当前位置开始
        AddCurrentDronePosition();
    }

    private void SetLineGradient()
    {
        // 创建一个颜色渐变，用于线的颜色随着时间推移而变化
        Gradient gradient = new Gradient();
        // 颜色键：从完全不透明的 lineColor 到完全透明的 lineColor
        GradientColorKey[] colorKeys = new GradientColorKey[2];
        colorKeys[0].color = lineColor;
        colorKeys[0].time = 0.0f; // 线的起点颜色
        colorKeys[1].color = lineColor;
        colorKeys[1].time = 1.0f; // 线的终点颜色

        // 透明度键：从完全不透明到完全透明
        GradientAlphaKey[] alphaKeys = new GradientAlphaKey[2];
        alphaKeys[0].alpha = lineColor.a; // 起点完全不透明
        alphaKeys[0].time = 0.0f;
        alphaKeys[1].alpha = 0.0f; // 终点完全透明
        alphaKeys[1].time = 1.0f;

        gradient.SetKeys(colorKeys, alphaKeys);
        lineRenderer.colorGradient = gradient;
    }

    // 添加当前无人机位置到轨迹点列表
    private void AddCurrentDronePosition()
    {
        Vector3 currentPosition = droneGameObject.transform.position;

        // 只有当无人机移动超过阈值时才添加新的轨迹点
        if (Vector3.Distance(currentPosition, lastAddedPosition) > minDistanceThreshold)
        {
            trajectoryPoints.Add(currentPosition);
            pointTimestamps.Add(Time.time); // 记录添加此点的时间
            lastAddedPosition = currentPosition;
        }
    }

    void Update()
    {
        if (droneGameObject == null) return; // 再次检查，防止运行时出错

        // 每帧检查并添加新的无人机位置点
        AddCurrentDronePosition();

        // 移除过期的轨迹点
        while (pointTimestamps.Count > 0 && Time.time - pointTimestamps[0] > maxLineLifetime)
        {
            trajectoryPoints.RemoveAt(0); // 移除最旧的位置点
            pointTimestamps.RemoveAt(0); // 移除最旧的时间戳
        }

        // 更新 LineRenderer
        if (trajectoryPoints.Count > 1) // 至少需要两个点才能画线
        {
            lineRenderer.positionCount = trajectoryPoints.Count;
            lineRenderer.SetPositions(trajectoryPoints.ToArray());
        }
        else
        {
            lineRenderer.positionCount = 0; // 如果点太少或没有点，则不显示线
        }
    }

    // 如果无人机被销毁或脚本停止，清理 LineRenderer
    void OnDisable()
    {
        if (lineRenderer != null)
        {
            lineRenderer.positionCount = 0; // 清空线
        }
    }
}
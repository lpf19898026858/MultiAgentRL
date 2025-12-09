// 文件名: RosTargetManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor; 
using RosMessageTypes.BuiltinInterfaces;
using System.Collections; // For Coroutines
using System.Collections.Generic; // For List

public class RosTargetManager : MonoBehaviour
{
    [Header("★ 无人机身份设置 ★")]
    [Tooltip("这个无人机的ROS命名空间/ID，例如 'V_UAV_0'。所有话题都会以此为前缀。")]
    public string uavNamespace = "drone"; // 提供一个默认值，但应在Inspector中为每个无人机设置

    [Header("ROS话题名称 (相对路径)")]
    // 这些是相对话题名，会在内部与 uavNamespace 组合
    public string targetRelativeTopic = "drone_target"; 
    public string statusRelativeTopic = "drone_status"; 
    public string poseRelativeTopic = "drone_pose";     
    public string imageRelativeTopic = "camera/image_raw";
    public string actionFeedbackRelativeTopic = "drone_action_feedback"; 

    [Header("摄像头图像发布设置")] 
    [Tooltip("拖拽你的Unity摄像机到这里")]
    public Camera imageCamera;
    [Tooltip("发布图像的频率 (Hz)")]
    public float imagePublishRateHz = 10f;
    [Tooltip("发布图像的宽度")]
    public int imageWidth = 640;
    [Tooltip("发布图像的高度")]
    public int imageHeight = 480;
    public string imageFrameId = "Unity_MainCamera";

    [Header("发布频率")]
    [Tooltip("状态和位姿的发布时间间隔（秒）")]
    public float poseStatusPublishInterval = 0.2f; // 每秒5次

    [Header("组件链接")]
    public PIDControlAgent agentToControl;
    public Transform targetVisual; // 用于可视化ROS目标

    private ROSConnection rosConnection; // 统一变量名
    private StringMsg statusMessage;
    private PoseStampedMsg poseMessage;
    private ImageMsg imageMessage; 
    private StringMsg actionFeedbackMessage; 
    private float timeSinceLastPoseStatusPublish;
    private float timeSinceLastImagePublish;
    private float imagePublishInterval;

    // 完整话题名 (包含命名空间) - 在Awake中初始化
    private string fullTargetTopic;
    private string fullStatusTopic;
    private string fullPoseTopic;
    private string fullImageTopic;
    private string fullActionFeedbackTopic;

    // 图像捕获专用资源
    private Texture2D texture2D;
    private RenderTexture renderTexture;
    private Rect cameraRect;
    private bool _publishersRegisteredAndReady = false; // Publisher就绪标志位

    // Awake 方法现在只负责获取 ROSConnection 实例，并进行一些初始化。
    void Awake() 
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        
        if (rosConnection == null)
        {
            Debug.LogError($"RosTargetManager for {uavNamespace}: ROSConnection instance is null in Awake(). Ensure ROSConnection GameObject exists and is configured.");
            enabled = false; // 禁用此脚本以避免更多错误
            return;
        }

        // --- 订阅 ROSConnection 的连接状态事件 (如果你在 ROSConnection.cs 中添加了这些事件) ---
        // 你的 ROSConnection.cs 源码没有 onConnected/onDisconnected 事件，所以暂时注释掉
        // 如果你修改了 ROSConnection.cs 源码添加了这些事件，可以取消注释
        // rosConnection.onConnected.AddListener(OnRosConnected);
        // rosConnection.onDisconnected.AddListener(OnRosDisconnected);

        // --- 构建完整话题名 (包含命名空间) ---
        fullTargetTopic         = $"/{uavNamespace}/{targetRelativeTopic}";
        fullStatusTopic         = $"/{uavNamespace}/{statusRelativeTopic}";
        fullPoseTopic           = $"/{uavNamespace}/{poseRelativeTopic}";
        fullImageTopic          = $"/{uavNamespace}/{imageRelativeTopic}";
        fullActionFeedbackTopic = $"/{uavNamespace}/{actionFeedbackRelativeTopic}";

        // --- 订阅话题 (使用完整话题名) ---
        rosConnection.Subscribe<PoseStampedMsg>(fullTargetTopic, OnTargetReceived);
        Debug.Log($"RosTargetManager: Subscribed to {fullTargetTopic} for {uavNamespace}");

        // --- 注册发布话题 (使用完整话题名) ---
        rosConnection.RegisterPublisher<StringMsg>(fullStatusTopic);
        rosConnection.RegisterPublisher<PoseStampedMsg>(fullPoseTopic); 
        rosConnection.RegisterPublisher<ImageMsg>(fullImageTopic); 
        rosConnection.RegisterPublisher<StringMsg>(fullActionFeedbackTopic); 
        Debug.Log($"RosTargetManager: Registered publishers for {uavNamespace}.");

        // --- 初始化消息对象 ---
        statusMessage = new StringMsg();
        poseMessage = new PoseStampedMsg();
        actionFeedbackMessage = new StringMsg();

        InitializeImagePublisher(); // 初始化图像相关的资源
        imagePublishInterval = 1f / imagePublishRateHz; // 计算图像发布间隔

        // 启动等待Publisher就绪的协程
        StartCoroutine(WaitForPublishersToBecomeReady());
        
        Debug.Log($"ROS连接已为 '{uavNamespace}' 设置。订阅: {fullTargetTopic}. 发布: {fullStatusTopic}, {fullPoseTopic}, {fullImageTopic}, {fullActionFeedbackTopic}");
    }

    void Start() // Start 方法只做一些最终检查
    {
        if (agentToControl == null || targetVisual == null) {
            Debug.LogError($"RosTargetManager for {uavNamespace}: Missing AgentToControl or TargetVisual link in Inspector!");
            enabled = false;
            return;
        }
        if (imageCamera == null) { 
            Debug.LogError($"RosTargetManager for {uavNamespace}: Image Camera 未分配！请拖拽一个摄像机到此字段。");
            enabled = false;
            return;
        }
    }

    // --- ROSConnection 连接状态回调 (如果 ROSConnection.cs 源码中没有这些事件，它们将无法被调用) ---
    /*
    private void OnRosConnected()
    {
        _publishersRegisteredAndReady = true;
        Debug.Log($"<color=green>RosTargetManager for {uavNamespace}: ROSConnection Established. Publishers are now active.</color>");
    }

    private void OnRosDisconnected()
    {
        _publishersRegisteredAndReady = false;
        Debug.LogWarning($"<color=red>RosTargetManager for {uavNamespace}: ROSConnection Disconnected. Publishers are inactive.</color>");
    }
    */

    private System.Collections.IEnumerator WaitForPublishersToBecomeReady()
    {
        float startTime = Time.time;
        float timeout = 10.0f; // 增加超时时间

        while (Time.time < startTime + timeout)
        {
            // <<< 修复 CS1061: 'ROSConnection' does not contain a definition for 'IsConnected' >>>
            // 依据 ROSConnection 源码，我们通过检查连接线程是否运行且无连接错误来判断连接状态
            // `HasConnectionThread` 表示连接线程已启动
            // `HasConnectionError` (静态变量) 表示最近一次连接尝试是否有错误
            if (rosConnection != null && rosConnection.HasConnectionThread && !rosConnection.HasConnectionError)
            {
                _publishersRegisteredAndReady = true;
                Debug.Log($"<color=green>RosTargetManager for {uavNamespace}: ROSConnection appears connected. Publishers are active.</color>");
                yield break; // 连接成功，就认为Publisher已就绪
            }
            // 只有当 rosConnection 实例存在时才尝试打印连接状态
            if (rosConnection != null)
            {
                Debug.LogWarning($"RosTargetManager for {uavNamespace}: Waiting for ROSConnection to establish connection. " +
                                 $"HasThread: {rosConnection.HasConnectionThread}, HasError: {rosConnection.HasConnectionError}");
            }
            yield return null; // 等待下一帧再检查
        }

        Debug.LogError($"<color=red>RosTargetManager for {uavNamespace}: Timeout waiting for ROSConnection to establish connection! Publishers will not be active.</color>");
        _publishersRegisteredAndReady = false; 
    }

    void FixedUpdate()
    {
        // 只有当 Publisher 准备就绪时才尝试发布
        if (!_publishersRegisteredAndReady) return; 

        // 位姿和状态的发布循环
        timeSinceLastPoseStatusPublish += Time.fixedDeltaTime; 
        if (timeSinceLastPoseStatusPublish >= poseStatusPublishInterval)
        {
            PublishStatus(); // 调用无参数的PublishStatus
            PublishPose();
            timeSinceLastPoseStatusPublish = 0f;
        }

        // 图像的独立发布循环
        timeSinceLastImagePublish += Time.fixedDeltaTime;
        if (timeSinceLastImagePublish >= imagePublishInterval)
        {
            PublishImage();
            timeSinceLastImagePublish = 0f;
        }
    }

    // --- ROS消息发布方法 ---
    void PublishStatus() 
    {
        // 即使 _publishersRegisteredAndReady 为 true，rosConnection也可能在运行时断开
        // 所以每次发布前都应检查 HasConnectionThread 和 HasConnectionError
        if (!_publishersRegisteredAndReady || !rosConnection.HasConnectionThread || rosConnection.HasConnectionError) return;
        
        string currentStatus = "UNKNOWN"; 

        if (agentToControl != null)
        {
            if (agentToControl.IsPerformingScriptedActionPublic)
            {
                currentStatus = "PERFORMING_ACTION";
            }
            else if (agentToControl.IsNavigatingPublic)
            {
                currentStatus = "NAVIGATING";
            }
            else if (agentToControl.IsAwaitingRosTargetPublic)
            {
                currentStatus = "HOVERING"; 
            }
            else
            {
                currentStatus = "IDLE";
            }
            
            if (agentToControl.transform.position.y < 0.2f && !agentToControl.IsNavigatingPublic && !agentToControl.IsPerformingScriptedActionPublic) {
                 currentStatus = "IDLE_ON_GROUND";
            }
        }
        else
        {
            currentStatus = "AGENT_NOT_LINKED"; 
            Debug.LogWarning($"RosTargetManager for {uavNamespace}: agentToControl is null when trying to PublishStatus.");
        }
        
        statusMessage.data = currentStatus;
        rosConnection.Publish(fullStatusTopic, statusMessage);
    }

    void PublishPose()
    {
        if (!_publishersRegisteredAndReady || !rosConnection.HasConnectionThread || rosConnection.HasConnectionError) return;
        
        Transform droneTransform = agentToControl.transform;

        poseMessage.header.stamp = new TimeMsg(); 
        poseMessage.header.frame_id = "world";

        // 填充位置 (Unity to ROS)
        poseMessage.pose.position.x = droneTransform.position.z;
        poseMessage.pose.position.y = -droneTransform.position.x;
        poseMessage.pose.position.z = droneTransform.position.y;
        
        // 填充姿态 (Unity to ROS)
        poseMessage.pose.orientation.x = -droneTransform.rotation.y;
        poseMessage.pose.orientation.y = droneTransform.rotation.z;
        poseMessage.pose.orientation.z = -droneTransform.rotation.x;
        poseMessage.pose.orientation.w = droneTransform.rotation.w;

        rosConnection.Publish(fullPoseTopic, poseMessage);
    }

    private void PublishImage()
    {
        if (!_publishersRegisteredAndReady || !rosConnection.HasConnectionThread || rosConnection.HasConnectionError) return;
        
        imageMessage.header.stamp = new TimeMsg(); 
        imageMessage.header.frame_id = imageFrameId; 

        imageCamera.targetTexture = renderTexture;
        imageCamera.Render();
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(cameraRect, 0, 0);
        texture2D.Apply();
        
        imageMessage.data = texture2D.GetRawTextureData();

        RenderTexture.active = null;
        imageCamera.targetTexture = null; 

        rosConnection.Publish(fullImageTopic, imageMessage);
    }

    public void PublishActionFeedback(string message)
    {
        if (!_publishersRegisteredAndReady || !rosConnection.HasConnectionThread || rosConnection.HasConnectionError) return;
        
        actionFeedbackMessage.data = message; 
        rosConnection.Publish(fullActionFeedbackTopic, actionFeedbackMessage); 
        Debug.Log($"<color=cyan>RosTargetManager: 已发布动作回报: '{message}' for {uavNamespace}</color>");
    }

    // --- ROS消息订阅回调 ---
    void OnTargetReceived(PoseStampedMsg msg)
    {
        Vector3 unityPosition = new Vector3(
            -(float)msg.pose.position.y, 
            (float)msg.pose.position.z, 
            (float)msg.pose.position.x
        );

        Quaternion unityRotation = new Quaternion(
            -(float)msg.pose.orientation.y,    
            (float)msg.pose.orientation.z,     
            -(float)msg.pose.orientation.x,    
            (float)msg.pose.orientation.w
        );

        Debug.Log($"<color=cyan>RosTargetManager for {uavNamespace}: 接收到新目标。Unity坐标: {unityPosition}, Unity旋转: {unityRotation.eulerAngles}°</color>");
        
        if (targetVisual != null) {
            targetVisual.position = unityPosition;
            targetVisual.rotation = unityRotation;
        } else {
            Debug.LogWarning($"RosTargetManager for {uavNamespace}: targetVisual is null, cannot update visual target.");
        }
        
        if (agentToControl != null) {
            agentToControl.SetTargetFromRos(unityPosition, unityRotation);
        } else {
            Debug.LogWarning($"RosTargetManager for {uavNamespace}: agentToControl is null, cannot set ROS target.");
        }
    }

    // --- 图像捕获专用资源初始化和清理 ---
    private void InitializeImagePublisher()
    {
        if (imageCamera == null) {
            Debug.LogError($"RosTargetManager for {uavNamespace}: Image Camera is null, cannot initialize image publisher resources.");
            enabled = false; 
            return;
        }

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        cameraRect = new Rect(0, 0, imageWidth, imageHeight);

        imageMessage = new ImageMsg
        {
            header = new HeaderMsg
            {
                frame_id = imageFrameId 
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageWidth * 3), 
            data = new byte[imageWidth * imageHeight * 3] 
        };

        imageCamera.targetTexture = renderTexture;
    }
    
    // 确保在对象销毁时释放资源
    private void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
            Destroy(renderTexture);
        }
        if (texture2D != null)
        {
            Destroy(texture2D);
        }
        
        // <<< 修复 CS1501: No overload for method 'Unsubscribe' takes 2 arguments >>>
        // Unsubscribe 只接受话题名称作为参数
        if (rosConnection != null) 
        {
            rosConnection.Unsubscribe(fullTargetTopic); 
            
            // 你的 ROSConnection.cs 源码没有 onConnected/onDisconnected 事件，所以这里移除对应的移除监听器代码
            // rosConnection.onConnected.RemoveListener(OnRosConnected);
            // rosConnection.onDisconnected.RemoveListener(OnRosDisconnected);
        }
    }
}
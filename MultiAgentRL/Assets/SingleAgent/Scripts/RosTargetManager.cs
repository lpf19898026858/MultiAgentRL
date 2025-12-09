// 文件名: RosTargetManager.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor; // <-- 新增: 引入Image消息的命名空间
using RosMessageTypes.BuiltinInterfaces;

public class RosTargetManager : MonoBehaviour
{
    [Header("ROS话题名称")]
    public string targetTopicName = "/drone_target";
    public string statusTopicName = "/drone_status";
    public string poseTopicName = "/drone_pose";
    public string imageTopicName = "/camera/image_raw"; 
    public string actionFeedbackTopic = "/drone_action_feedback"; 

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
    public Transform targetVisual;

    private ROSConnection ros;
    private StringMsg statusMessage;
    private PoseStampedMsg poseMessage; // 用于发布位姿
    private ImageMsg imageMessage; 
    private StringMsg actionFeedbackMessage; 
    private float timeSinceLastPoseStatusPublish;
    private float timeSinceLastImagePublish;
    private float imagePublishInterval;
    private bool isTaskActive = false;

    // 图像捕获专用资源
    private Texture2D texture2D;
    private RenderTexture renderTexture;
    private Rect cameraRect;

    void Start()
    {
        if (agentToControl == null || targetVisual == null) { /* ... 错误处理 ... */ return; }
        if (imageCamera == null) { 
            Debug.LogError("Image Camera 未分配！请拖拽一个摄像机到此字段。");
            return;
        }
        ros = ROSConnection.GetOrCreateInstance();
        
        // 订阅目标话题
        ros.Subscribe<PoseStampedMsg>(targetTopicName, OnTargetReceived);
        
        // 注册发布话题
        ros.RegisterPublisher<StringMsg>(statusTopicName);
        ros.RegisterPublisher<PoseStampedMsg>(poseTopicName); 
        ros.RegisterPublisher<ImageMsg>(imageTopicName);
        ros.RegisterPublisher<StringMsg>(actionFeedbackTopic); // <<< 新增：注册回报发布者
        
        // --- 初始化消息 ---
        statusMessage = new StringMsg();
        poseMessage = new PoseStampedMsg();
        actionFeedbackMessage = new StringMsg(); // <<< 新增：初始化回报消息
        InitializeImagePublisher(); 

        // --- 计算发布间隔 ---
        imagePublishInterval = 1f / imagePublishRateHz;
        
        statusMessage = new StringMsg();
        poseMessage = new PoseStampedMsg(); // <-- 新增: 初始化位姿消息
        
        Debug.Log($"ROS连接已设置。订阅: {targetTopicName}. 发布: {statusTopicName}, {poseTopicName}, {imageTopicName}");
    }

void FixedUpdate()
{
        // 位姿和状态的发布循环
        timeSinceLastPoseStatusPublish += Time.fixedDeltaTime; 
        if (timeSinceLastPoseStatusPublish >= poseStatusPublishInterval)
        {
            PublishStatus();
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
    public void PublishActionFeedback(string message)
    {
        actionFeedbackMessage.data = message;
        ros.Publish(actionFeedbackTopic, actionFeedbackMessage);
        Debug.Log($"<color=cyan>RosTargetManager: 已发布动作回报: '{message}'</color>");
    }
    private void InitializeImagePublisher()
    {
        // 创建一次性的资源
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24, RenderTextureFormat.ARGB32);
        cameraRect = new Rect(0, 0, imageWidth, imageHeight);

        // 初始化ROS Image消息结构
        imageMessage = new ImageMsg
        {
            header = new HeaderMsg
            {
                frame_id = imageFrameId
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8", // 纯RGB字节
            is_bigendian = 0,
            step = (uint)(imageWidth * 3), // 每行字节数: 宽度 * 3 (RGB)
            data = new byte[imageWidth * imageHeight * 3]
        };
    }

    private void PublishImage()
    {
        // 1. 更新时间戳
        imageMessage.header.stamp = new TimeMsg();

        // 2. 核心图像捕获逻辑
        imageCamera.targetTexture = renderTexture;
        imageCamera.Render();
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(cameraRect, 0, 0);
        texture2D.Apply();
        
        // 3. 将像素数据填充到消息中
        imageMessage.data = texture2D.GetRawTextureData();

        // 4. 清理
        RenderTexture.active = null;
        imageCamera.targetTexture = null;

        // 5. 发布消息
        ros.Publish(imageTopicName, imageMessage);
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
    }


void OnTargetReceived(PoseStampedMsg msg)
    {
        // 1. 从ROS消息中提取并转换位置 (ROS -> Unity)
        Vector3 unityPosition = new Vector3(
            -(float)msg.pose.position.y, 
            (float)msg.pose.position.z, 
            (float)msg.pose.position.x
        );

        // 2. 新增: 从ROS消息中提取并转换姿态/旋转 (ROS -> Unity)
        Quaternion unityRotation = new Quaternion(
            -(float)msg.pose.orientation.y, 
            (float)msg.pose.orientation.z, 
            -(float)msg.pose.orientation.x, 
            (float)msg.pose.orientation.w
        );

        Debug.Log($"<color=cyan>接收到新目标。Unity坐标: {unityPosition}, Unity旋转: {unityRotation.eulerAngles}°</color>");
        
        // 更新用于视觉显示的目标对象
        targetVisual.position = unityPosition;
        targetVisual.rotation = unityRotation;
        
        isTaskActive = true;
        
        // 3. 修改: 调用正确的方法名，并传入位置和旋转两个参数nt)
        agentToControl.SetTargetFromRos(unityPosition, unityRotation);
    }

    void PublishStatus()
    {
        string currentStatus = "UNKNOWN"; // 默认状态

        // --- 核心状态决策逻辑 ---
        if (agentToControl.IsPerformingScriptedAction)
        {
            // 最高优先级：如果正在执行脚本动作，状态就是动作本身。
            // (这里可以进一步细化，比如从Agent获取具体动作名，但暂时用一个通用状态)
            currentStatus = "PERFORMING_ACTION";
        }
        else if (agentToControl.IsNavigating)
        {
            // 第二优先级：如果没有执行脚本动作，且正在导航，那么状态就是导航中。
            currentStatus = "NAVIGATING";
        }
        else if (agentToControl.IsAwaitingRosTarget)
        {
            // 第三优先级：如果既没执行动作，也没在导航，且在等待指令，那就是悬停等待。
            currentStatus = "HOVERING"; // 或者 "IDLE_IN_AIR"
        }
        else
        {
            // 其他情况，可以认为是通用IDLE
            currentStatus = "IDLE";
        }

        // 可以添加一个最终的覆盖逻辑，比如根据高度判断是否在地面上
        if (agentToControl.transform.position.y < 0.2f && !agentToControl.IsNavigating) {
             currentStatus = "IDLE_ON_GROUND";
        }
        
        statusMessage.data = currentStatus;
        ros.Publish(statusTopicName, statusMessage);
    }

    // <-- 新增: 发布位姿的完整方法 -->
void PublishPose()
{
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

    ros.Publish(poseTopicName, poseMessage);
}
}
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Random = UnityEngine.Random;
using Unity.MLAgents.Policies;
using System.Collections.Generic; 
using System.IO; 
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using System;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System.Threading.Tasks;
using System.Net.Http;
using System.Text;
using Newtonsoft.Json;
using RosMessageTypes.NlpDroneControl;

public struct RosTarget
{
    public Vector3 position;
    public Quaternion rotation;
}

public class PIDControlAgent : Agent
{
    #region 
    private float maxTargetPitch = 45f;
    private float maxTargetRoll = 45f;
    private float maxTargetVelocityX = 5f; 
    private float maxTargetVelocityZ = 5f; 
    private float maxYawRate = 90f; 
    private float maxTargetAltitude = 10f; 
    private float maxVerticalVelocityForAltitudePID = 3f;

    [Header("底层速度 & 姿态PID飞控设置")]
    public PIDController velocityXPID = new PIDController { Kp = 0.5f, Ki = 0.0f, Kd = 0.1f }; // 将前向速度误差转换为俯仰角
    public PIDController velocityZPID = new PIDController { Kp = 0.5f, Ki = 0.0f, Kd = 0.1f }; // 将侧向速度误差转换为横滚角
    public PIDController velocityYPID = new PIDController { Kp = 2.5f, Ki = 0.5f, Kd = 0.5f }; // **调高参数，使其更积极地稳定垂直速度**
    public PIDController pitchController = new PIDController { Kp = 6f, Ki = 0f, Kd = 0.4f };
    public PIDController rollController = new PIDController { Kp = 6f, Ki = 0f, Kd = 0.4f };
    public PIDController yawController = new PIDController { Kp = 1.0f, Ki = 0f, Kd = 0f }; // 现在是偏航角度控制器
    public PIDController altitudeController = new PIDController { Kp = 0.8f, Ki = 0.1f, Kd = 0.2f };

    [Header("物理属性")]
    public Rigidbody rBody;
    [Tooltip("用于抵消重力的基础推力乘数。在考虑倾斜补偿后，可以设为1.0。")]
    public float hoverThrustMultiplier = 1.0f;

    [Header("核心奖励设置")]
    public float arrivalReward = 1.0f;           // 成功的最终奖励
    public float terminalFailurePenalty = -1.0f; // 所有导致结束的失败的统一惩罚
    [Tooltip("成功到达时的最大着陆奖金")]
    public float landingBonus = 0.2f;

    [Header("核心塑形奖励 (引导Agent行为)")]
    [Tooltip("朝向目标移动的基础奖励乘数")]
    public float potentialRewardMultiplier = 0.1f;
    [Tooltip("在“终端区”内，对准目标的奖励乘数")]
    public float terminalAlignmentBonus = 0.001f;
    [Tooltip("在“终端区”内，控制速度的奖励乘数")]
    public float terminalSpeedControlBonus = 0.002f;
    [Tooltip("在“进近区”内，对准方向的奖励乘数")]
    public float approachDirectionBonus = 0.0005f;
    [Tooltip("当需要绕障时，奖励横向移动的奖励值")]
    public float evasiveManeuverBonus = 0.01f;

    [Header("动态奖励区域 (与课程挂钩)")]
    [Range(0.1f, 0.9f)]
    [Tooltip("将局部目标范围的多大比例作为“进近区”")]
    public float approachZoneRatio = 0.8f;
    [Range(0.1f, 0.9f)]
    [Tooltip("将局部目标范围的多大比例作为“终端区”")]
    public float terminalZoneRatio = 0.4f;
    public float arrivalZoneDistance = 1.0f; // 到达区可以保持固定

    [Header("辅助性惩罚 (数值应非常小)")]
    public float timePenaltyPerStep = -0.0001f;
    public float excessiveAngularVelocityPenalty = -0.001f;
    public float maxAllowedAngularVelocity = 45.0f;
    public float stagnationPenalty = -0.001f;
    [Tooltip("惩罚对着障碍物发呆的行为")]
    public float obstacleStagnationPenalty = -0.005f;
    public float actionChangePenalty = -0.00005f;
    public float energyConsumptionPenalty = -0.00002f;
    public float obstacleProximityPenalty = -0.002f;
    public float minSafeObstacleDistance = 1.5f;

    [Header("环境设置")]
    [Tooltip("无人机出生范围的逻辑中心点。如果留空，将使用此物体在场景中的初始位置。")]
    public Transform trainingArea; 
    public Transform spawnAreaCenter;
    public LayerMask groundLayer;
    public LayerMask obstacleLayer;

    private float minClearanceFromObstacles = 2.0f;

    private float minX = -32f, maxX = 40f;
    private float minY = 0f, maxY = 6f; // 无人机能飞行的最小和最大高度
    private float minZ = -45f, maxZ = 40f;

    private float _targetVelocityX; // Agent的目标前向/后向速度
    private float _targetVelocityZ; // Agent的目标侧向速度
    private float _targetYawRate;   // Agent的目标偏航角速度
    private float _targetAltitude;  // Agent的目标高度
    public Transform target;

    // 内部控制变量 (由速度PID计算得出，作为姿态PID的设定点)
    private float _currentDesiredYawAngle; // 累积Agent的目标偏航角
    private float _desiredPitchAngle;      // 由 velocityXPID 计算得出的目标俯仰角
    private float _desiredRollAngle;       // 由 velocityZPID 计算得出的目标横滚角

    // 使用静态变量，以便场景中所有Agent实例共享这些计数器
    private static int totalEpisodes = 0;
    private static int successfulEpisodes = 0;
    private StatsRecorder statsRecorder; // 用于将自定义统计数据发送到TensorBoard
    
    private int stepReportInterval = 50000;
    private static int totalStepCounter = 0; // 用于追踪总步数
    private static int windowTotalEpisodes = 0;      // 窗口期内的总回合数
    private static int windowSuccessfulEpisodes = 0; // 窗口期内的成功回合数
    private static int lastReportedStep = 0;         // 上一次报告时的步数

    private Vector3 initialPosition;
    private Vector3 _currentRosTargetPosition;
    private float lastDistanceToTarget; 

    private static string logFilePath;
    private static bool isLogInitialized = false;
    private static float lastKnownRangeX = -1.0f;
    private static float lastKnownRangeZ = -1.0f;
    private static float lastKnownLocalRange = -1.0f;
    private static int lessonCounter = 0; // 用于给课程编号
    private Vector3 currentEpisodeStartPosition; 

    private bool _isAwaitingNextRosTarget = false;
    private float? _rosTargetYaw = null;
    private Vector3 _gizmoCenterForLocalRange;

    private Queue<RosTarget> _rosTargetQueue = new Queue<RosTarget>();
    private RayPerceptionSensorComponent3D _raySensor;

    private float _originalMass;
    private float _originalDrag;
    private float _originalAngularDrag;

    private float _lastTargetVelocityX;
    private float _lastTargetVelocityZ;
    [Range(0, 0.95f)] // 限制范围，防止完全不响应
    [Tooltip("平滑Agent的动作指令，值越高越平滑，但响应越慢。")]
    public float actionSmoothing = 0.5f;

    [Header("程序化内容生成 (PCG)")]
    [Tooltip("勾选后，将在每个训练回合动态生成随机障碍物。")]
    public bool enablePCG = true;
    [Tooltip("用于随机生成障碍物的 Prefab 列表。")]
    public List<GameObject> obstaclePrefabs;
    private List<GameObject> _generatedObstacles = new List<GameObject>();

    private Vector3 _nextWaypointPosition;
    private bool _hasNextWaypoint;
    private float _currentSpawnRangeX;
    private float _currentSpawnRangeZ;
    private float _currentLocalTargetRange;
    private bool _isPcgActiveForThisLesson; // 用于缓存本回合是否激活PCG
    
    // 导航任务是否激活？
    public bool IsNavigating { get; private set; } = false;
    public bool IsNavigatingPublic => IsNavigating; 
    // 是否正在执行一个非导航的脚本化动作？
    public bool IsPerformingScriptedActionPublic => _isPerformingScriptedAction; // 公共只读属性

    // 是否正在等待ROS指令？（悬停等待状态）
    public bool IsAwaitingRosTargetPublic => _isAwaitingNextRosTarget; // 公共只读属性
    
    // 我们需要一个标志位来告诉OnActionReceived何时忽略ONNX模型
    private bool _isPerformingScriptedAction = false;
    private Quaternion? _scriptedTargetRotation = null;

    [Header("模式设置")]
    public bool useRosForInference = true;
    public Transform cameraTransform;
    public RosTargetManager rosManager;

    private float inspectionRotationDuration = 2.0f;
    private bool isRosInferenceMode;
    private bool _isFlyingToFinalDestination; // 新增：是否正在飞往最终目的地
    private readonly Queue<System.Collections.IEnumerator> _actionQueue = new Queue<System.Collections.IEnumerator>();
    private Coroutine _currentActionCoroutine = null; // 用于跟踪当前正在执行的协程

private string logPrefix;

    #endregion
public override void Initialize()
    {
        rBody = GetComponent<Rigidbody>();
        initialPosition = transform.position;
        statsRecorder = Academy.Instance.StatsRecorder;
        if (!isLogInitialized)
        {
            // 定义日志文件路径，存储在项目根目录下的 "Logs" 文件夹中
            string logDirectory = Path.Combine(Application.dataPath, "..", "Logs");
            Directory.CreateDirectory(logDirectory); // 如果文件夹不存在，则创建
            logFilePath = Path.Combine(logDirectory, $"FailureLog_{System.DateTime.Now:yyyy-MM-dd_HH-mm-ss}.txt");

            // 写入文件头
            File.WriteAllText(logFilePath, "--- Drone Failure Log ---\n\n");
            Log($"Failure log file initialized at: {logFilePath}");
            isLogInitialized = true;
        }

         if (spawnAreaCenter == null)
        {
            LogWarning($"'Spawn Area Center' 未设置，将使用Agent的初始位置 ({initialPosition}) 作为生成区域的中心。");
            var centerObject = new GameObject($"{name}_SpawnCenter");
            centerObject.transform.position = initialPosition;
            spawnAreaCenter = centerObject.transform;
        }
        if (cameraTransform == null)
        {
            LogError("请在Inspector中为检查序列指定Camera Transform！");
        }
        if (rBody == null)
        {
            LogError("Rigidbody component not found on the drone agent!");
        }
        rBody.useGravity = true;

        _originalMass = rBody.mass;
        _originalDrag = rBody.drag;
        _originalAngularDrag = rBody.angularDrag;

        logPrefix = $"<b>[{gameObject.name}]</b>";
        if (rosManager != null && !string.IsNullOrEmpty(rosManager.uavNamespace))
        {
            string serviceTopic = $"/{rosManager.uavNamespace}/execute_drone_action"; // 根据无人机命名空间生成服务话题
            ROSConnection.GetOrCreateInstance().ImplementService<ExecuteDroneActionRequest, ExecuteDroneActionResponse>(serviceTopic, ExecuteActionService);
            Log($"已注册 ROS 服务: {serviceTopic}");
        }
        else
        {
            LogError($"PIDControlAgent {name}: rosManager未设置或其uavNamespace为空，无法注册ROS服务。");
        }
        StartCoroutine(ActionQueueProcessor());
    }
private void Log(string message)
{
    // 在原始消息前加上前缀
    Debug.Log($"{logPrefix} {message}");
}

private void LogWarning(string message)
{
    Debug.LogWarning($"{logPrefix} {message}");
}

private void LogError(string message)
{
    Debug.LogError($"{logPrefix} {message}");
}
public override void OnEpisodeBegin()
{
    // --- 步骤 1: 获取环境参数并重置基本状态 ---
    _currentSpawnRangeX = Academy.Instance.EnvironmentParameters.GetWithDefault("target_spawn_range_x", 5.0f);
    _currentSpawnRangeZ = Academy.Instance.EnvironmentParameters.GetWithDefault("target_spawn_range_z", 5.0f);
    _currentLocalTargetRange = Academy.Instance.EnvironmentParameters.GetWithDefault("local_target_range", 5.0f);
    
    _isPcgActiveForThisLesson = (_currentSpawnRangeX > 10.0f || _currentSpawnRangeZ > 10.0f);
    isRosInferenceMode = !Academy.Instance.IsCommunicatorOn && useRosForInference;

    IsNavigating = false;
    _isPerformingScriptedAction = false;
    _isAwaitingNextRosTarget = isRosInferenceMode;
    //_rosTargetQueue.Clear();
    
    rBody.velocity = Vector3.zero;
    rBody.angularVelocity = Vector3.zero;

    Vector3 startPos;

    // --- 步骤 2: 根据模式（训练 vs 推理）执行完全独立的初始化流程 ---
    if (isRosInferenceMode)
    {
        rBody.mass = _originalMass;
        rBody.drag = _originalDrag;
        rBody.angularDrag = _originalAngularDrag;
        
        CleanUpObstacles();

        startPos = transform.position;
        currentEpisodeStartPosition = startPos;
        _currentRosTargetPosition = currentEpisodeStartPosition;
        target.position = _currentRosTargetPosition;
        
        _isAwaitingNextRosTarget = true;
        _hasNextWaypoint = false;
        _isFlyingToFinalDestination = true;
        
        Log("<color=purple>推理模式: 回合开始。在起点悬停并等待ROS目标。</color>");
    }
    else 
    {
        // a. 随机化物理属性
        rBody.mass = _originalMass * Random.Range(0.9f, 1.1f);
        rBody.drag = _originalDrag * Random.Range(0.9f, 1.1f);
        rBody.angularDrag = _originalAngularDrag * Random.Range(0.9f, 1.1f);

        // b. 确定出生点
        startPos = GetSafeCurriculumStartPosition();
        currentEpisodeStartPosition = startPos;
        transform.position = startPos;
        transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
        
        // c. 确定目标点
        _currentRosTargetPosition = GetSafeRandomTargetPosition(startPos);
        target.position = _currentRosTargetPosition;
        
        // d. 围绕目标点生成障碍物
        if (enablePCG && _isPcgActiveForThisLesson) 
        {
            // 使用与目标生成范围一致的参数
            GenerateRandomObstacles(_currentRosTargetPosition, _currentLocalTargetRange / 2.0f,  _currentLocalTargetRange / 2.0f
    );
        }
        else
        {
            CleanUpObstacles();
        }
        
        // e. 随机任务类型 (单目标 vs 多航点)
        bool isMultiWaypointScenario = Random.value > 0.5f;
        if (isMultiWaypointScenario)
        {
            Log("<color=cyan>训练回合: 模拟多航点任务 (冲刺模式)</color>");
            Vector3 nextPosCandidate = _currentRosTargetPosition + new Vector3(Random.Range(-5f, 5f), 0, Random.Range(-5f, 5f));
            
            // 修正后的安全检查逻辑: CheckSphere返回true意味着有碰撞，不安全
            if (Physics.CheckSphere(nextPosCandidate, minClearanceFromObstacles, obstacleLayer))
            {
                _nextWaypointPosition = _currentRosTargetPosition;
            }
            else
            {
                _nextWaypointPosition = nextPosCandidate;
            }
            _hasNextWaypoint = true;
            _isFlyingToFinalDestination = false;
        }
        else
        {
            Log("<color=orange>训练回合: 模拟单目标任务 (刹车模式)</color>");
            _hasNextWaypoint = false;
            _isFlyingToFinalDestination = true;
        }
                // 只有在训练模式下，才清空所有任务队列
        _rosTargetQueue.Clear();
        lock (_actionQueue)
        {
            _actionQueue.Clear();
        }
        _isAwaitingNextRosTarget = false; // 训练时立即开始
    }

    // 重置所有控制器和状态变量
    _rosTargetYaw = null;
    _currentDesiredYawAngle = GetYaw(); 
    lastDistanceToTarget = Vector3.Distance(transform.position, _currentRosTargetPosition);
    
    pitchController.Reset();
    rollController.Reset();
    yawController.Reset(); 
    altitudeController.Reset();
    velocityYPID.Reset();
    velocityXPID.Reset();
    velocityZPID.Reset();

    _targetAltitude = startPos.y; 
    _targetVelocityX = 0f;
    _targetVelocityZ = 0f;
    _desiredPitchAngle = 0f;
    _desiredRollAngle = 0f;
    
    _gizmoCenterForLocalRange = startPos; 
    _lastTargetVelocityX = 0f;
    _lastTargetVelocityZ = 0f;
}

private void CleanUpObstacles()
{
    if (_generatedObstacles != null)
    {
        foreach (var obstacle in _generatedObstacles)
        {
            Destroy(obstacle);
        }
        _generatedObstacles.Clear();
    }
}
private ExecuteDroneActionResponse ExecuteActionService(ExecuteDroneActionRequest request)
{
    Log($"<color=lightblue>Received and QUEUED action service request: {request.action_type}</color>");
    var jsonSettings = new JsonSerializerSettings
    {
        Culture = System.Globalization.CultureInfo.InvariantCulture
    };
    System.Collections.IEnumerator actionCoroutine = null;
    bool is_known_action = true;
    float duration;

    // 步骤 1: 根据请求类型，创建对应的协程实例，但先不启动它
    switch (request.action_type)
    {
        case "adjust_camera_pitch":
            var cam_params = JsonConvert.DeserializeObject<Dictionary<string, float>>(request.params_json);
            float pitchAngle = cam_params["pitch_angle"];
            actionCoroutine = AnimateCameraPitchRoutine(pitchAngle, inspectionRotationDuration);
            break;

        case "rotate_drone_yaw_relative":
            var yaw_params = JsonConvert.DeserializeObject<Dictionary<string, float>>(request.params_json);
            float relativeAngle = yaw_params["angle_degrees"];
            actionCoroutine = RotateYawRelativeRoutine(relativeAngle);
            break;

        case "backup":
            var backup_params = JsonConvert.DeserializeObject<Dictionary<string, float>>(request.params_json);
            float distance = backup_params.ContainsKey("distance") ? backup_params["distance"] : 2.0f;
            duration = backup_params.ContainsKey("duration") ? backup_params["duration"] : 2.5f;
            actionCoroutine = BackupRoutine(distance, duration);
            break;

        case "wait":
            var wait_params = JsonConvert.DeserializeObject<Dictionary<string, float>>(request.params_json);
            duration = wait_params.ContainsKey("duration") ? wait_params["duration"] : 5.0f;
            actionCoroutine = WaitRoutine(duration);
            break;

        case "takeoff":
            var takeoff_params = JsonConvert.DeserializeObject<Dictionary<string, float>>(request.params_json);
            float targetAltitude = takeoff_params.ContainsKey("altitude") ? takeoff_params["altitude"] : 3.0f;
            actionCoroutine = TakeoffRoutine(targetAltitude);
            break;

        case "land":
            actionCoroutine = LandRoutine();
            break;

        default:
            LogError($"Unknown action type received: {request.action_type}");
            is_known_action = false;
            break;
    }

    // 步骤 2: 如果动作是已知的，就将其加入队列
    if (is_known_action && actionCoroutine != null)
    {
        lock (_actionQueue) // 锁定队列以保证线程安全
        {
            _actionQueue.Enqueue(actionCoroutine);
        }
    }
    
    return new ExecuteDroneActionResponse
    {
        success = is_known_action,
        message = is_known_action ? "Action queued successfully." : "Unknown action type."
    };
}
private System.Collections.IEnumerator ActionQueueProcessor()
{
    Log("Action Queue Processor started.");
    while (true) // 无限循环
    {
        // 步骤 1: 等待，直到无人机处于可以执行脚本动作的状态
        // 这个状态意味着：既没有在导航，也没有在执行另一个脚本动作。
        while (IsNavigating || _isPerformingScriptedAction)
        {
            // 如果正在导航或执行动作，就等待一小段时间再检查
            yield return new WaitForSeconds(0.2f); 
        }

        // 步骤 2: 检查队列中是否有待执行的动作
        System.Collections.IEnumerator actionToExecute = null;
        lock (_actionQueue) // 锁定队列以保证线程安全
        {
            if (_actionQueue.Count > 0)
            {
                // 从队列中取出一个动作
                actionToExecute = _actionQueue.Dequeue();
            }
        }
        
        // 步骤 3: 如果取出了动作，就执行它
        if (actionToExecute != null)
        {
            Log($"<color=lime>Starting next action from queue. Drone is idle.</color>");
            // 使用 PerformScriptedAction 包装器来管理状态。
            // ActionQueueProcessor会在这里等待，直到这个协程执行完毕。
            yield return StartCoroutine(PerformScriptedAction(actionToExecute));
        }
        else
        {
            // 如果队列是空的，也等待一小段时间再进入下一次大循环
            yield return new WaitForSeconds(0.1f);
        }
    }
}
    private System.Collections.IEnumerator AnimateCameraPitchRoutine(float targetPitchAngle, float duration)
    {
        if (cameraTransform == null)
        {
            LogError("无法旋转摄像头：cameraTransform未设置！");
            yield break; // 提前退出协程
        }

        Quaternion startRotation = cameraTransform.localRotation;
        Quaternion targetRotation = Quaternion.Euler(targetPitchAngle, 0, 0);
        float elapsedTime = 0f;

        while (elapsedTime < duration)
        {
            cameraTransform.localRotation = Quaternion.Slerp(startRotation, targetRotation, elapsedTime / duration);
            elapsedTime += Time.deltaTime;
            yield return null;
        }
        cameraTransform.localRotation = targetRotation;
    }

    private System.Collections.IEnumerator WaitRoutine(float duration)
    {
        Log($"Executing WaitRoutine for {duration} seconds.");
    
        // 核心逻辑就是等待
        yield return new WaitForSeconds(duration);
    
        Log("Wait finished.");
    }
/// <summary>
/// 通过PID控制器将无人机旋转到绝对目标偏航角。
/// 这个协程会一直运行，直到无人机的角度和角速度都满足稳定条件。
/// </summary>
private System.Collections.IEnumerator AnimateDroneYawRoutine(float targetYawAngle, float rotationSpeedFactor = 1.0f)
{
    Log($"<color=yellow>命令无人机旋转至绝对Yaw: {targetYawAngle}°</color>");
    
    _scriptedTargetRotation = Quaternion.Euler(0, NormalizeAngle(targetYawAngle), 0);
    
    float safetyTimeout = 10.0f; 
    float elapsedTime = 0f;

    float stabilizationTime = 0.3f; // 要求稳定的持续时间
    float timeStable = 0f;          // 已经持续稳定的时间
    float angleTolerance = 2.5f;    // 角度容差 (可以稍微放宽一点)
    float angularVelocityTolerance = 1.0f; // 角速度容差 (单位: 度/秒)

    while (elapsedTime < safetyTimeout) 
    {
        float currentYaw = GetYaw();
        // 获取Y轴的角速度，并从弧度/秒转换为度/秒
        float currentAngularVelocityY = rBody.angularVelocity.y * Mathf.Rad2Deg;

        // 条件1: 角度是否在容差范围内?
        bool angleIsOk = Mathf.Abs(Mathf.DeltaAngle(currentYaw, targetYawAngle)) < angleTolerance;
        
        // 条件2: 角速度是否足够低 (即，是否已经基本停止旋转)?
        bool rotationIsStopped = Mathf.Abs(currentAngularVelocityY) < angularVelocityTolerance;

        // 必须同时满足两个条件，才算是“可能稳定”
        if (angleIsOk && rotationIsStopped)
        {
            // 如果两个条件都满足，开始/继续稳定计时
            timeStable += Time.deltaTime;
        }
        else
        {
            // 只要有一个条件不满足，就重置计时器，从头开始等待稳定
            timeStable = 0f;
        }

        // 只有当稳定时间足够长，我们才真正确信它稳定了
        if (timeStable >= stabilizationTime)
        {
            Log($"<color=green>已到达并稳定在目标偏航角 {targetYawAngle}° (实际: {currentYaw:F2}, 角速度: {currentAngularVelocityY:F2} deg/s)</color>");
            yield break; // 成功完成，退出协程
        }
        
        elapsedTime += Time.deltaTime;
        yield return null; // 等待下一帧
    }

    if (elapsedTime >= safetyTimeout)
    {
        LogError($"旋转超时！未能到达目标偏航角 {targetYawAngle}°。当前角度: {GetYaw()}, 角速度: {rBody.angularVelocity.y * Mathf.Rad2Deg:F2} deg/s");
    }
}
    
    /// <summary>
    /// 一个通用的协程包装器，用于管理脚本化动作的生命周期。
    /// 它负责设置和清除 _isPerformingScriptedAction 标志位。
    /// </summary>
    private System.Collections.IEnumerator PerformScriptedAction(System.Collections.IEnumerator actionCoroutine)
    {
    // 动作开始前的状态设置 (这部分是正确的)
    _isAwaitingNextRosTarget = false; // 明确我们在忙，不是在等待
    IsNavigating = false;
    _isPerformingScriptedAction = true;

    // 在执行任何脚本动作前，重置所有可能相关的PID控制器，
    // 特别是那个被遗忘的 yawController。
    // 这能确保每次动作都从一个“干净”的初始状态开始，消除历史误差累积。
    pitchController.Reset();
    rollController.Reset();
    yawController.Reset(); 
    
    // 同时，强制将速度目标清零，防止速度控制器干扰
    _targetVelocityX = 0f;
    _targetVelocityZ = 0f;
    _targetYawRate = 0f;

    _scriptedTargetRotation = null; // 开始时清空临时旋转目标
    _rosTargetYaw = null; 
    
    yield return StartCoroutine(actionCoroutine); // 执行具体的动作协程
    _currentDesiredYawAngle = NormalizeAngle(GetYaw());
    yield return new WaitForSeconds(0.5f); // 0.5秒稳定时间

        if (rosManager != null)
        {
            rosManager.PublishActionFeedback("ACTION_COMPLETE");
        }
        else
        {
            LogWarning("无法发送动作完成回报，因为 rosManager 未设置。");
        }

    _isPerformingScriptedAction = false; // 动作完成，归还控制权
    _scriptedTargetRotation = null; // 结束后清空临时旋转目标
    
    // 动作结束后，立即检查队列并决定下一步该做什么
    if (_rosTargetQueue.Count > 0)
    {
        // 如果队列里还有任务，立即开始下一个导航
        ProcessNextTargetInQueue(); 
    }
    else
    {
        // 如果队列空了，才进入等待状态
        IsNavigating = false; 
        _isAwaitingNextRosTarget = true;
    }
    }

    private System.Collections.IEnumerator AdjustCameraRoutine(float targetPitch)
    {
        Log($"Executing AdjustCameraRoutine to pitch: {targetPitch} degrees.");
        // 在这里添加你实际控制相机GameObject旋转的代码
        // 例如:
        // Transform cameraTransform = ...;
        // float currentPitch = cameraTransform.localEulerAngles.x;
        // float elapsedTime = 0f;
        // float duration = 2.0f;
        // while(elapsedTime < duration)
        // {
        //     cameraTransform.localEulerAngles = new Vector3(
        //         Mathf.Lerp(currentPitch, targetPitch, elapsedTime / duration), 0, 0);
        //     elapsedTime += Time.deltaTime;
        //     yield return null;
        // }
        
        yield return new WaitForSeconds(2.0f); // 作为一个简单的占位符，我们等待2秒
        Log("Camera adjustment finished.");
    }

private System.Collections.IEnumerator RotateYawRelativeRoutine(float relativeAngleDegrees)
{
    Log($"接收到相对旋转命令: {relativeAngleDegrees}°");
    float startYaw = GetYaw();
    float targetYaw = startYaw + relativeAngleDegrees;
    
    yield return StartCoroutine(AnimateDroneYawRoutine(targetYaw));
}

    private void Hover()
    {
        // 将所有目标速度和角速度设为0，PID控制器会自动将无人机稳定下来。
        _targetVelocityX = 0f;
        _targetVelocityZ = 0f;
        _targetYawRate = 0f;
        // _targetAltitude 保持不变，以维持当前高度
        _currentDesiredYawAngle = GetYaw();
    
    }

    private System.Collections.IEnumerator TakeoffRoutine(float targetAltitude)
    {
        Log($"Executing TakeoffRoutine to altitude: {targetAltitude}m.");
        
        // 命令无人机PID控制器将目标高度设为指定值
        _targetAltitude = targetAltitude;
        
        // 等待无人机实际到达目标高度附近 (留出一点容错空间)
        // 这个循环会持续多帧，直到条件满足
        while (transform.position.y < targetAltitude - 0.1f)
        {
            yield return null; // 等待下一物理帧
        }
        
        Log("Takeoff finished. Hovering at target altitude.");
    }

    private System.Collections.IEnumerator LandRoutine()
    {
        // 命令无人机PID控制器将目标高度设为一个非常接近地面的值
        _targetAltitude = 0.3f;
        
        // 等待无人机实际降低到地面附近
        while (transform.position.y > 0.2f)
        {
            yield return null; // 等待下一物理帧
        }
        
        // (可选) 落地后可以进一步稳定，比如将速度强制归零
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;

        Log("Landing finished. Drone is on the ground.");
    }

private System.Collections.IEnumerator BackupRoutine(float distance, float duration)
{
    Vector3 startPos = transform.position;
    Vector3 targetPos = startPos - (transform.forward * distance);

    float elapsedTime = 0f;
    while(elapsedTime < duration)
    {
        transform.position = Vector3.Lerp(startPos, targetPos, elapsedTime / duration);
        elapsedTime += Time.deltaTime;
        yield return null;
    }
    
    transform.position = targetPos;
    if (rBody != null)
    {
        rBody.velocity = Vector3.zero;
        rBody.angularVelocity = Vector3.zero;
    }
}
    private void ProcessNextTargetInQueue()
    {
        if (_rosTargetQueue.Count > 0)
        {
            RosTarget nextTarget = _rosTargetQueue.Dequeue();

            _currentRosTargetPosition = nextTarget.position;
            target.position = nextTarget.position; // 更新可视化目标
            _rosTargetYaw = NormalizeAngle(nextTarget.rotation.eulerAngles.y);

            _isAwaitingNextRosTarget = false; // 我们现在正在积极飞向一个目标。
            IsNavigating = true; // <--- 关键：开始导航时，设置状态为 true
            _gizmoCenterForLocalRange = transform.position;
            Log($"<color=lime>正在处理队列中的下一个目标。位置: {_currentRosTargetPosition}。 " +
                      $"偏航角: {_rosTargetYaw.Value}°。队列中还剩 {_rosTargetQueue.Count} 个目标。</color>");

            lastDistanceToTarget = Vector3.Distance(transform.position, _currentRosTargetPosition);
            pitchController.Reset();
            rollController.Reset();
            yawController.Reset();
            altitudeController.Reset();
            velocityYPID.Reset();
            velocityXPID.Reset();
            velocityZPID.Reset();
            _targetVelocityX = 0f; 
            _targetVelocityZ = 0f; 
            if (_rosTargetQueue.Count > 0)
            {
                nextTarget = _rosTargetQueue.Peek();
                _nextWaypointPosition = nextTarget.position;
                _hasNextWaypoint = true;
            }
            else
            {
                _hasNextWaypoint = false;
            }
        }
        else
        {
            // 队列已空。所有航点均已到达。
            _isAwaitingNextRosTarget = true; // 进入悬停/等待状态。
            IsNavigating = false; // <--- 关键：导航队列为空，设置状态为 false
            Log("<color=cyan>航点队列已空。正在悬停并等待新的ROS目标。</color>");
            _rosTargetYaw = null; 
        }
    }

    public void SetTargetFromRos(Vector3 newTargetPosition, Quaternion newTargetRotation)
    {
        // 只在推理模式下接受目标。
        if (Academy.Instance.IsCommunicatorOn || !useRosForInference)
        {
            return;
        }
        if (target == null)
        {
            LogError("致命错误！'target' transform 为空。中止 SetTargetFromRos。");
            return;
        }
        if (newTargetRotation.x == 0 && newTargetRotation.y == 0 && newTargetRotation.z == 0 && newTargetRotation.w == 0)
        {
            LogWarning("SetTargetFromRos: 收到一个无效的(零)四元数。已修正为单位四元数。");
            newTargetRotation = Quaternion.identity;
        }

        var newTarget = new RosTarget { position = newTargetPosition, rotation = newTargetRotation };
        _rosTargetQueue.Enqueue(newTarget);
        
        Log($"<color=yellow>新目标已入队。位置: {newTarget.position}。 " +
                  $"队列中总目标数: {_rosTargetQueue.Count}。</color>");

        if (_isAwaitingNextRosTarget)
        {
            _isAwaitingNextRosTarget = false; 
            ProcessNextTargetInQueue();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // 自身状态
        sensor.AddObservation(transform.InverseTransformDirection(rBody.angularVelocity)); // 3 - 自身坐标系下的角速度
        sensor.AddObservation(transform.InverseTransformDirection(rBody.velocity));    // 3 - 自身坐标系下的线速度 (Z:forward, X:right, Y:up)
        sensor.AddObservation(transform.localRotation);                                // 4 - 姿态 (Quaternion)

        // 2. 到 *当前* 目标的信息 (4维)
        Vector3 dirToCurrentTargetWorld = _currentRosTargetPosition - transform.position;
        sensor.AddObservation(transform.InverseTransformDirection(dirToCurrentTargetWorld.normalized)); // 3
        sensor.AddObservation(dirToCurrentTargetWorld.magnitude); // 1 (使用 magnitude 更直接)
        // 在 CollectObservations() 的末尾添加
        sensor.AddObservation(_targetVelocityX / maxTargetVelocityX); // 归一化到 -1到1
        sensor.AddObservation(_targetVelocityZ / maxTargetVelocityZ); // 归一化到 -1到1
        sensor.AddObservation(_targetYawRate / maxYawRate);           // 归一化到 -1到1
        // 对于高度，考虑目标高度是否也需要归一化到0-1，以当前高度范围(minY到maxY)为基准
        sensor.AddObservation((_targetAltitude - minY) / (maxY - minY)); // 归一化到 0到1

        // 4. 到 *下一个* 航点的信息
        Vector3 nextWaypointDirectionLocal;
        float nextWaypointDistance;

        if (_hasNextWaypoint)
        {
            // 如果存在下一个航点，计算其相对信息
            Vector3 dirToNextWaypointWorld = _nextWaypointPosition - transform.position;
            nextWaypointDirectionLocal = transform.InverseTransformDirection(dirToNextWaypointWorld.normalized);
            nextWaypointDistance = dirToNextWaypointWorld.magnitude;
        }
        else
        {
            // 如果这是最后一个点，就用当前目标的信息来填充，表示“下一步就是这里”
            nextWaypointDirectionLocal = transform.InverseTransformDirection(dirToCurrentTargetWorld.normalized);
            nextWaypointDistance = dirToCurrentTargetWorld.magnitude;
        }
        sensor.AddObservation(nextWaypointDirectionLocal); // 3
        sensor.AddObservation(nextWaypointDistance);       // 1
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (_isPerformingScriptedAction)
        {
            Hover(); // 在执行脚本动作时，底层保持悬停
            return;
        }

        if (isRosInferenceMode && _isAwaitingNextRosTarget)
        {
            // 这里可以调用一个专门的悬停动作函数，或者直接设置目标为0
            // ApplyHoverAction(); // 如果你有这个函数
            _targetVelocityX = 0f;
            _targetVelocityZ = 0f;
            _targetYawRate = 0f;
            Hover(); 
            // _targetAltitude 保持不变，以实现悬停
            return; // 在等待时，不执行神经网络的动作
        }
            // 将核心逻辑委托给新的 ApplyActions 方法
            ApplyActions(actions.ContinuousActions);
    }

private void ApplyActions(ActionSegment<float> continuousActions)
{
    // === 1. 从神经网络获取原始、不平滑的目标 ===
    // 这是神经网络在当前帧认为“最理想”的动作
    float rawTargetVelX = continuousActions[0] * maxTargetVelocityX;
    float rawTargetVelZ = continuousActions[1] * maxTargetVelocityZ;
    float rawTargetYawRate = continuousActions[2] * maxYawRate;
    float rawTargetAltitude = Mathf.Lerp(minY, maxY, (continuousActions[3] + 1) / 2f);

    // === 2. 对原始目标进行平滑处理 (所有模式下都生效) ===
    // 使用 Lerp 进行低通滤波，得到一个更稳定、延迟的基础目标。
    // 这能极大地减少神经网络输出高频抖动带来的问题。
    // 新的目标 = Lerp(上一步的目标, 这一步的原始目标, 平滑权重)
    // 注意: Time.deltaTime 可以让平滑效果与帧率无关，但简单的 Lerp 通常也够用。
    // 为了简单起见，我们直接用 1-actionSmoothing 作为权重。
    _targetVelocityX = Mathf.Lerp(_targetVelocityX, rawTargetVelX, 1f - actionSmoothing);
    _targetVelocityZ = Mathf.Lerp(_targetVelocityZ, rawTargetVelZ, 1f - actionSmoothing);
    _targetYawRate   = Mathf.Lerp(_targetYawRate,   rawTargetYawRate,   1f - actionSmoothing);
    _targetAltitude  = Mathf.Lerp(_targetAltitude,  rawTargetAltitude,  1f - actionSmoothing);

    // === 3. 在平滑后的基础上，仅在训练时添加动作噪声 ===
    // 这确保了我们的探索是建立在一个稳定的基线之上。
    if (Academy.Instance.IsCommunicatorOn) // 只在训练时添加噪声
    {
        float noiseStrength = 0.1f; // 这个值也可以做成可配置的参数
        
        // 在平滑后的速度上添加噪声
        _targetVelocityX += Random.Range(-1f, 1f) * noiseStrength * maxTargetVelocityX;
        _targetVelocityZ += Random.Range(-1f, 1f) * noiseStrength * maxTargetVelocityZ;

        // (可选) 你也可以对其他动作添加噪声
        // _targetYawRate += Random.Range(-1f, 1f) * noiseStrength * maxYawRate;
    }
    // === 4. 更新偏航角 ===
    // 这里的逻辑保持不变
    _currentDesiredYawAngle += _targetYawRate * Time.fixedDeltaTime;
    if (_currentDesiredYawAngle > 180f) _currentDesiredYawAngle -= 360f;
    if (_currentDesiredYawAngle < -180f) _currentDesiredYawAngle += 360f;
}


private void CalculatedAndApplyRewards()
{
    float distanceToTarget = Vector3.Distance(transform.position, _currentRosTargetPosition);
    if (distanceToTarget < arrivalZoneDistance) 
    {
        if (!isRosInferenceMode)
        {
            float finalSpeed = rBody.velocity.magnitude;
            float finalLandingBonus = Mathf.Exp(-finalSpeed) * landingBonus;

            float totalArrivalReward = arrivalReward + finalLandingBonus;
            AddReward(totalArrivalReward);
            
            totalEpisodes++; successfulEpisodes++; windowTotalEpisodes++; windowSuccessfulEpisodes++;
            RecordSuccessRate();
            
            EndEpisode();
            return;
        }
    }

    if (CheckBoundsAndEndEpisode())
    {
        return; 
    }
    // --- 2a. 核心“拉力”：势能奖励 (始终有效) ---
    // 这是最基础的奖励，鼓励Agent不断靠近目标。
    float distanceImprovement = lastDistanceToTarget - distanceToTarget;
    AddReward(distanceImprovement * potentialRewardMultiplier);
    lastDistanceToTarget = distanceToTarget;

    // --- 2. 区分终点和中途点的塑形奖励 ---
    Vector3 droneToTargetDir = (_currentRosTargetPosition - transform.position).normalized;

    if (_isFlyingToFinalDestination)
    {
        // --- 策略 A: 飞向【最终目的地】，鼓励精确、平稳地“降落” ---
        float dynamicTerminalDistance = _currentLocalTargetRange * terminalZoneRatio;
        if (distanceToTarget <= dynamicTerminalDistance)
        {
            // 奖励1: 对准目标
            float alignment = Vector3.Dot(transform.forward, droneToTargetDir);
            if (alignment > 0)
            {
                AddReward(alignment * alignment * terminalAlignmentBonus);
            }

            // 奖励2: 控制速度，鼓励减速
            float idealSpeed = distanceToTarget * (maxTargetVelocityX / _currentLocalTargetRange); 
            idealSpeed = Mathf.Clamp(idealSpeed, 0, maxTargetVelocityX);
            float speedError = Mathf.Abs(rBody.velocity.magnitude - idealSpeed);
            float speedReward = Mathf.Exp(-speedError) * terminalSpeedControlBonus;
            AddReward(speedReward);
        }
            if (rBody.velocity.magnitude < 0.2f && distanceToTarget > dynamicTerminalDistance) 
    {
        AddReward(stagnationPenalty);
    }
    }
    else
    {
        // --- 策略 B: 飞向【中途航点】，鼓励保持速度、对准飞行方向 ---
        // 我们不希望它在中途点减速，所以这里不使用终端区奖励
        float dynamicApproachDistance = _currentLocalTargetRange * approachZoneRatio;
        if (distanceToTarget <= dynamicApproachDistance && rBody.velocity.magnitude > 0.5f) // 速度大于0.5才给奖励
        {
            // 奖励1: 飞行方向与目标方向一致
            float directionMatch = Vector3.Dot(rBody.velocity.normalized, droneToTargetDir);
            if (directionMatch > 0)
            {
                AddReward(directionMatch * approachDirectionBonus);
            }
            
            // (可选) 奖励2: 鼓励保持较高速度通过中途点
            // float speedBonus = (rBody.velocity.magnitude / maxTargetVelocityX) * 0.0001f;
            // AddReward(speedBonus);
        }
            if (rBody.velocity.magnitude < 0.2) 
    {
        AddReward(stagnationPenalty);
    }
    AddReward(-0.001f);
    }
    // --- 2c. 智能绕障引导 ---
    Vector3 dirToTargetLocal = transform.InverseTransformDirection(droneToTargetDir);
    bool isObstacleInFront = Physics.SphereCast(transform.position, 0.5f, transform.forward, out RaycastHit hit, 3f, obstacleLayer);

    if (dirToTargetLocal.z > 0.1f && isObstacleInFront) // 目标在前方且有障碍物
    {
        float lateralVelocity = Mathf.Abs(rBody.velocity.x); // 自身坐标系下的横向速度
        if (lateralVelocity > 0.5f)
        {
            AddReward(evasiveManeuverBonus); // 奖励横向绕行
        }
        else
        {
            AddReward(obstacleStagnationPenalty); // 惩罚对着障碍物发呆
        }
    }

    // --- 2d. 通用微调与惩罚 ---
    Collider[] hitColliders = Physics.OverlapSphere(transform.position, minSafeObstacleDistance, obstacleLayer);
    if (hitColliders.Length > 0)
    {
        float closestDist = float.MaxValue;
        foreach (var col in hitColliders)
        {
            Vector3 closestPoint = col.ClosestPoint(transform.position);
            closestDist = Mathf.Min(closestDist, Vector3.Distance(transform.position, closestPoint));
        }
        float normalizedProximity = Mathf.InverseLerp(minSafeObstacleDistance, 0f, closestDist);
        AddReward(normalizedProximity * normalizedProximity * obstacleProximityPenalty);
    }

    AddReward(timePenaltyPerStep);
    
    if (rBody.angularVelocity.magnitude * Mathf.Rad2Deg > maxAllowedAngularVelocity)
    {
        AddReward(excessiveAngularVelocityPenalty);
    }
    
    float velocityChange = Mathf.Abs(_targetVelocityX - _lastTargetVelocityX) + Mathf.Abs(_targetVelocityZ - _lastTargetVelocityZ);
    AddReward(velocityChange * actionChangePenalty);
    _lastTargetVelocityX = _targetVelocityX;
    _lastTargetVelocityZ = _targetVelocityZ;

    float speedSquared = rBody.velocity.sqrMagnitude;
    AddReward(speedSquared * energyConsumptionPenalty);
}

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions.Clear();

        if (Input.GetKey(KeyCode.W)) continuousActions[0] = 1f;
        else if (Input.GetKey(KeyCode.S)) continuousActions[0] = -1f;
        else continuousActions[0] = 0f;

        if (Input.GetKey(KeyCode.D)) continuousActions[1] = 1f; // Right
        else if (Input.GetKey(KeyCode.A)) continuousActions[1] = -1f; // Left
        else continuousActions[1] = 0f;

        if (Input.GetKey(KeyCode.E)) continuousActions[2] = 1f; // Yaw Right
        else if (Input.GetKey(KeyCode.Q)) continuousActions[2] = -1f; // Yaw Left
        else continuousActions[2] = 0f;

        if (Input.GetKey(KeyCode.LeftShift)) continuousActions[3] = 1f; // 目标更高
        else if (Input.GetKey(KeyCode.LeftControl)) continuousActions[3] = -1f; // 目标更低
        else continuousActions[3] = 0f; // 保持目标高度不变
    }

    private void CheckAndLogLessonChange()
    {
        float currentRangeX = Academy.Instance.EnvironmentParameters.GetWithDefault("target_spawn_range_x", 5.0f);
        float currentRangeZ = Academy.Instance.EnvironmentParameters.GetWithDefault("target_spawn_range_z", 5.0f);
        float currentLocalRange = Academy.Instance.EnvironmentParameters.GetWithDefault("local_target_range", 5.0f);
        if (Mathf.Abs(currentRangeX - lastKnownRangeX) > 0.01f || Mathf.Abs(currentRangeZ - lastKnownRangeZ) > 0.01f||Mathf.Abs(currentLocalRange - lastKnownLocalRange) > 0.01f)
        {
            lastKnownRangeX = currentRangeX;
            lastKnownRangeZ = currentRangeZ;
            lastKnownLocalRange = currentLocalRange;
            lessonCounter++;
            string lessonMarker = $"\n=============== LESSON {lessonCounter} STARTED ===============\n" +
                                  $"Global Range: X={currentRangeX:F1}, Z={currentRangeZ:F1}\n" +
                                  $"Local Target Range: {currentLocalRange:F1}\n" +
                                  $"============================================\n";
            try
            {
                File.AppendAllText(logFilePath, lessonMarker);
            }
            catch (IOException ex)
            {
                LogError($"Error writing lesson marker to log file: {ex.Message}");
            }

            Log($"<color=yellow>Lesson changed to {lessonCounter}. New Range: X={currentRangeX:F1}, Z={currentRangeZ:F1}</color>");
        }
    }
void FixedUpdate()
{    // 只有在推理模式下，并且当前有任务（不是在等待第一个任务）时，才检查是否到达
    if (isRosInferenceMode && IsNavigating)
    {
        // 计算无人机与当前航点的距离
        float distanceToCurrentWaypoint = Vector3.Distance(transform.position, _currentRosTargetPosition);

        // 如果距离小于到达阈值 (例如1.0米)
        if (distanceToCurrentWaypoint < arrivalZoneDistance) // 使用你已经定义的arrivalZoneDistance
        {
            Log($"<color=green>到达航点: {_currentRosTargetPosition}。正在处理队列中的下一个目标...</color>");
            
            // 立即处理队列中的下一个航点
            // 这会更新 _currentRosTargetPosition 为下一个航点，或者如果队列为空，则将 _isAwaitingNextRosTarget 设为 true
            ProcessNextTargetInQueue();
        }
    }
        CheckAndLogLessonChange();
        totalStepCounter++;
        if (rBody)
        {
            ExecuteFlightController();
            CalculatedAndApplyRewards();
            CheckAndReportWindowedSuccessRate();
        }
        if (StepCount >= MaxStep && MaxStep > 0) // MaxStep>0 确保这个功能是开启的
        {
            //SetReward(0.0f);
            Log($"<color=orange>达到最大步数 {MaxStep}！回合超时结束。最终奖励: 0.0</color>");
            RecordFailurePoint(transform.position, "Timeout");
            totalEpisodes++;
            windowTotalEpisodes++;
            RecordSuccessRate(); // 这个函数内部会计算成功率，所以未成功的也需要在这里更新分母
            EndEpisode();
        }

}

    private void ExecuteFlightController()
    {
        float distanceToTarget = Vector3.Distance(transform.position, _currentRosTargetPosition);
    
        // 从RL模型获取的目标速度 (先保存一份原始值)
        float agentTargetVelocityX = _targetVelocityX;
        float agentTargetVelocityZ = _targetVelocityZ;

        float currentPitch = GetPitch();
        float currentRoll = GetRoll();
        float currentYaw = GetYaw();
        Vector3 currentLocalVelocity = transform.InverseTransformDirection(rBody.velocity); // 获取无人机自身坐标系下的线速度 (重要!)

        // --- 2. 水平速度控制 (外环) -> 姿态角目标 (内环的设定点) ---
        // 前向/后向速度控制俯仰角 (Unity的 Z 轴是 forward)
        // 为了向前飞行 (_targetVelocityX 为正)，无人机需要俯冲（负的俯仰角），所以 PID 结果需要取反。
        _desiredPitchAngle = -velocityXPID.Calculate(_targetVelocityX, currentLocalVelocity.z, Time.fixedDeltaTime);
        // 侧向速度控制横滚角 (Unity的 X 轴是 right)
        // 为了向右飞行 (_targetVelocityZ 为正)，无人机需要右倾（正的横滚角），所以 PID 结果不需要取反。
        _desiredRollAngle = velocityZPID.Calculate(_targetVelocityZ, currentLocalVelocity.x, Time.fixedDeltaTime);

        // 限制计算出的目标姿态角在物理允许范围内 (使用你原来的 maxTargetPitch/Roll 作为物理限制)
        _desiredPitchAngle = -Mathf.Clamp(_desiredPitchAngle, -maxTargetPitch, maxTargetPitch);
        _desiredRollAngle = Mathf.Clamp(_desiredRollAngle, -maxTargetRoll, maxTargetRoll);

        // --- 3. 姿态控制 (内环) -> 扭矩 ---
        // 使用水平速度PID计算出的 _desiredPitchAngle 和 _desiredRollAngle 作为设定点
        float pitchTorque = pitchController.Calculate(_desiredPitchAngle, currentPitch, Time.fixedDeltaTime);
        float rollTorque = rollController.Calculate(_desiredRollAngle, currentRoll, Time.fixedDeltaTime);

        float yawTorque;
        if (_isPerformingScriptedAction && _scriptedTargetRotation.HasValue)
        {
            float scriptedYawSetpoint = NormalizeAngle(_scriptedTargetRotation.Value.eulerAngles.y);
    
            // ★★★ 使用 Mathf.DeltaAngle 来计算最短路径误差 ★★★
            float yawError = Mathf.DeltaAngle(currentYaw, scriptedYawSetpoint);
            yawTorque = yawController.Calculate(0, -yawError, Time.fixedDeltaTime); // PID的目标是误差为0
        }
        else
        {
            // 【正常导航/训练模式】: 使用原来的逻辑
            float yawSetpoint;
            if (_rosTargetYaw.HasValue && !Academy.Instance.IsCommunicatorOn)
            {
                yawSetpoint = _rosTargetYaw.Value;
            }
            else
            {
                yawSetpoint = _currentDesiredYawAngle;
            }
            // ★★★ 这里也使用 Mathf.DeltaAngle ★★★
            float yawError = Mathf.DeltaAngle(currentYaw, yawSetpoint);
            yawTorque = yawController.Calculate(0, -yawError, Time.fixedDeltaTime); // PID的目标是误差为0
        }

        rBody.AddTorque(transform.right * pitchTorque);
        rBody.AddTorque(transform.forward * -rollTorque); 
        rBody.AddTorque(transform.up * yawTorque);

        // --- 5. 垂直推力计算 (高度-速度级联控制) ---
        // 基础悬停推力，抵消重力
        float hoverThrust = rBody.mass * Physics.gravity.magnitude * hoverThrustMultiplier;

        // **外环: 高度控制器 (altitudeController) 计算所需的目标垂直速度**
        // 输入：Agent设定的目标高度 (_targetAltitude) 和当前实际高度 (transform.localPosition.y)
        float targetVerticalVelocity = altitudeController.Calculate(_targetAltitude, transform.localPosition.y, Time.fixedDeltaTime);
        // 限制由高度PID计算出的目标垂直速度，防止在高度误差大时生成过大的垂直速度指令
        targetVerticalVelocity = Mathf.Clamp(targetVerticalVelocity, -maxVerticalVelocityForAltitudePID, maxVerticalVelocityForAltitudePID);

        // **内环: 垂直速度控制器 (velocityYPID) 根据目标垂直速度和当前垂直速度，计算推力修正**
        // 输入：外环生成的目标垂直速度 (targetVerticalVelocity) 和当前实际垂直速度 (currentLocalVelocity.y)
        float thrustCorrection = velocityYPID.Calculate(targetVerticalVelocity, currentLocalVelocity.y, Time.fixedDeltaTime);

        // 倾斜补偿：当无人机倾斜时，垂直向上的推力会减小，需要增加总推力来维持高度
        float tiltAngle = Vector3.Angle(transform.up, Vector3.up); // 当前无人机向上方向与世界坐标系向上方向的夹角
        float tiltCompensation = 1f / Mathf.Max(0.01f, Mathf.Cos(tiltAngle * Mathf.Deg2Rad)); // cos(0)=1, cos(90)=0

        float finalThrust = (hoverThrust + thrustCorrection) * tiltCompensation; // 使用内环PID输出的修正量
        rBody.AddForce(transform.up * finalThrust);
    }

void Update()
{
    // 按下 'R' 键，命令无人机旋转到 90 度
    if (Input.GetKeyDown(KeyCode.R))
    {
        Log("Test command: Rotating to 90 degrees!");
        // 启动你的旋转协程，就像ROS调用一样
        StartCoroutine(PerformScriptedAction(RotateYawRelativeRoutine(90.0f)));
    }

    // 按下 'T' 键，命令无人机旋转到 -45 度
    if (Input.GetKeyDown(KeyCode.T))
    {
        Log("Test command: Rotating to -45 degrees!");
        StartCoroutine(PerformScriptedAction(RotateYawRelativeRoutine(-45.0f)));
    }
}
private void OnCollisionEnter(Collision collision)
{
    string failureReason = "";
    bool didFail = false;

    if (((1 << collision.gameObject.layer) & obstacleLayer) != 0)
    {
        failureReason = $"Hit Obstacle: {collision.gameObject.name}";
        didFail = true;
    }
    else if (((1 << collision.gameObject.layer) & groundLayer) != 0)
    {
        failureReason = "Hit Ground";
        didFail = true;
    }
    if (didFail)
    {
        AddReward(terminalFailurePenalty); 
        Log($"<color=red>碰撞失败! {failureReason}。最终奖励: {terminalFailurePenalty}</color>");
        
        RecordFailurePoint(_currentRosTargetPosition, failureReason);

        totalEpisodes++;
        windowTotalEpisodes++;
        RecordSuccessRate();
        if(!isRosInferenceMode)
        {
            EndEpisode();
        }
    }
}

private bool CheckBoundsAndEndEpisode()
{
    Vector3 currentPos = transform.localPosition;
    if (currentPos.x < minX || currentPos.x > maxX ||
        currentPos.y < minY || currentPos.y > maxY ||
        currentPos.z < minZ || currentPos.z > maxZ)
    {
        AddReward(terminalFailurePenalty);
        Log($"<color=red>飞出边界! 最终奖励: {terminalFailurePenalty}</color>");
        RecordFailurePoint(currentPos, "Out of Bounds"); // 记录失败
        
        totalEpisodes++;
        windowTotalEpisodes++;
        RecordSuccessRate();
        if(!isRosInferenceMode)
        {
            EndEpisode();
        }
        return true; 
    }
    return false; 
}

    private void RecordFailurePoint(Vector3 position, string reason)
    {
        // 将Vector3坐标格式化为字符串，例如 "(1.23, 4.56, 7.89)"
        string positionString = $"({position.x:F2}, {position.y:F2}, {position.z:F2})";
        string logEntry = $"Target: {positionString} - Reason: {reason}\n";
        
        // 使用 File.AppendAllText 来追加内容，这会自动处理文件的打开和关闭
        try
        {
            File.AppendAllText(logFilePath, logEntry);
        }
        catch (IOException ex)
        {
            LogError($"Error writing to log file: {ex.Message}");
        }
    }
private void CheckAndReportWindowedSuccessRate()
{
    if (totalStepCounter >= lastReportedStep + stepReportInterval)
    {
        float windowedSuccessRate = 0f;
        if (windowTotalEpisodes > 0)
        {
            windowedSuccessRate = (float)windowSuccessfulEpisodes / windowTotalEpisodes;
        }
        statsRecorder.Add("Custom/Windowed Success Rate", windowedSuccessRate);

        windowTotalEpisodes = 0;
        windowSuccessfulEpisodes = 0;
        
        lastReportedStep = totalStepCounter;
    }
}
    private float GetPitch(){float pitch = transform.localEulerAngles.x;if (pitch > 180f) { pitch -= 360f; }return pitch;}
    private float GetRoll(){float roll = transform.localEulerAngles.z;if (roll > 180f) { roll -= 360f; }return -roll; }
    private float GetYaw(){float yaw = transform.localEulerAngles.y;if (yaw > 180f) { yaw -= 360f; }return yaw;}
    private float NormalizeAngle(float angle){while (angle > 180f) angle -= 360f;while (angle < -180f) angle += 360f;return angle;}
private void RecordSuccessRate()
{
    if (totalEpisodes > 0)
    {
        float successRate = (float)successfulEpisodes / totalEpisodes;
        statsRecorder.Add("Custom/Success Rate", successRate);
    }
}

    private Vector3 GetSafeCurriculumStartPosition()
    {
        Vector3 randomPos;
        int attempts = 0;
        Vector3 targetInitialPoint;
        do
        {
            targetInitialPoint=trainingArea.position;
            float randX = Random.Range(targetInitialPoint.x, targetInitialPoint.x + _currentSpawnRangeX);
            float randZ = Random.Range(targetInitialPoint.z, targetInitialPoint.z + _currentSpawnRangeZ);

            randomPos = new Vector3(
                randX,
                Random.Range(minY, maxY),
                randZ
            );

            randomPos.x = Mathf.Clamp(randomPos.x, minX + minClearanceFromObstacles, maxX - minClearanceFromObstacles);
            randomPos.y = Mathf.Clamp(randomPos.y, minY + minClearanceFromObstacles, maxY - minClearanceFromObstacles); // 也要考虑无人机飞行的最小高度
            randomPos.z = Mathf.Clamp(randomPos.z, minZ + minClearanceFromObstacles, maxZ - minClearanceFromObstacles);

            attempts++;
            if (!Physics.CheckSphere(randomPos, minClearanceFromObstacles, obstacleLayer))
            {
                return randomPos;
            }

        } while (attempts < 100);

        LogError("无法为无人机找到一个安全的出生点！请检查课程学习范围、障碍物层或环境边界。将返回中心点。");
        return targetInitialPoint;
    }

    private Vector3 GetSafeRandomTargetPosition(Vector3 startPosition)
    {
        Vector3 newPos;
        int attempts = 0;

        float minTargetX = startPosition.x - _currentLocalTargetRange / 2;
        float maxTargetX = startPosition.x + _currentLocalTargetRange / 2;
        float minTargetZ = startPosition.z - _currentLocalTargetRange / 2;
        float maxTargetZ = startPosition.z + _currentLocalTargetRange / 2;

        do
        {
            newPos = new Vector3(
                Random.Range(minTargetX, maxTargetX),
                Random.Range(minY, maxY), // 高度在整个允许范围内随机
                Random.Range(minTargetZ, maxTargetZ)
            );

            newPos.x = Mathf.Clamp(newPos.x, minX, maxX);
            newPos.y = Mathf.Clamp(newPos.y, minY, maxY);
            newPos.z = Mathf.Clamp(newPos.z, minZ, maxZ);

            attempts++;

            // 检查目标点是否离起始点太近，并确保目标点本身是安全的
            if (Vector3.Distance(newPos, startPosition) > 3.0f && !Physics.CheckSphere(newPos, minClearanceFromObstacles, obstacleLayer))
            {
                return newPos;
            }

        } while (attempts < 100);

        LogWarning($"无法在 {startPosition} 附近找到安全的目标点。将返回起始点前方5米处作为备用目标。");
        return startPosition + transform.forward * 5f;
    }
private void GenerateRandomObstacles(Vector3 centerPoint, float rangeX, float rangeZ)
{
    CleanUpObstacles(); // 先清理旧的
    //int numberOfObstacles = Random.Range(1, 3); // Random.Range(min, max) 中 max 是不包含的, 所以 (1, 3) 会返回 1 或 2
    int numberOfObstacles = 1;
    // 循环生成障碍物
    for (int i = 0; i < numberOfObstacles; i++)
    {
        // 从预制体列表中随机选一个
        if (obstaclePrefabs == null || obstaclePrefabs.Count == 0)
        {
            LogError("障碍物预制体列表 (obstaclePrefabs) 为空！");
            return;
        }
        GameObject prefabToSpawn = obstaclePrefabs[Random.Range(0, obstaclePrefabs.Count)];

        //在指定范围内计算随机位置
        Vector3 randomPosition;
        int maxAttempts = 20; // 设置一个尝试上限，防止死循环
        int attempts = 0;
        bool positionIsSafe = false;

        do
        {
            float randomX = Random.Range(-rangeX, rangeX);
            float randomZ = Random.Range(-rangeZ, rangeZ);

            randomPosition = new Vector3(centerPoint.x + randomX, 
                                          centerPoint.y, // 通常障碍物和目标在同一高度
                                          centerPoint.z + randomZ);
            
            // 检查这个位置是否安全
            // 我们不希望障碍物生成得离目标点太近，或者互相重叠
            float minDistanceToTarget = 3.0f; // 障碍物距离目标点的最小距离
            float minDistanceToOtherObstacles = 2.0f; // 障碍物之间的最小距离
            
            // 检查与目标点的距离
            if (Vector3.Distance(randomPosition, centerPoint) < minDistanceToTarget)
            {
                attempts++;
                continue; // 离目标太近，重新生成
            }

            // 检查与其他已生成障碍物的距离
            bool tooCloseToOthers = false;
            foreach(var existingObstacle in _generatedObstacles)
            {
                if (Vector3.Distance(randomPosition, existingObstacle.transform.position) < minDistanceToOtherObstacles)
                {
                    tooCloseToOthers = true;
                    break;
                }
            }
            if (tooCloseToOthers)
            {
                attempts++;
                continue; // 离其他障碍物太近，重新生成
            }
            
            // 如果所有检查都通过了
            positionIsSafe = true;

        } while (!positionIsSafe && attempts < maxAttempts);

        // --- 6. 实例化障碍物 ---
        if (positionIsSafe)
        {
            GameObject newObstacle = Instantiate(prefabToSpawn, randomPosition, Quaternion.Euler(0, Random.Range(0f, 360f), 0));
            // (可选) 你可以给障碍物设置随机大小
            // newObstacle.transform.localScale = Vector3.one * Random.Range(0.8f, 1.5f);
            
            _generatedObstacles.Add(newObstacle); // 加入列表以便下次清理
        }
        else
        {
            LogWarning("无法为障碍物找到一个安全的位置，跳过生成。");
        }
    }
}
    void OnDrawGizmos()
    {
        if (target != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(target.position, 1.0f);
        }
        Gizmos.color = Color.red;
        Vector3 boundsCenter = new Vector3((minX + maxX) / 2f, (minY + maxY) / 2f, (minZ + maxZ) / 2f);
        Vector3 boundsSize = new Vector3(maxX - minX, maxY - minY, maxZ - minZ);
        Gizmos.DrawWireCube(boundsCenter, boundsSize);
        //if (!Application.isPlaying) return;
 
        Vector3 targetInitialPoint=trainingArea.position;
        Gizmos.color = Color.cyan;
        Vector3 targetSpawnSize = new Vector3(
            _currentSpawnRangeX, 
            maxY - minY, 
            _currentSpawnRangeZ  
        );
        Vector3 targetSpawnCenter = new Vector3(
            targetInitialPoint.x + targetSpawnSize.x / 2f, 
            minY + targetSpawnSize.y / 2f,   
            targetInitialPoint.z + targetSpawnSize.z / 2f    
        );
        Gizmos.DrawWireCube(targetSpawnCenter, targetSpawnSize);
        
        Gizmos.color = Color.yellow;
        Vector3 targetAreaCenter = _gizmoCenterForLocalRange; 
        targetAreaCenter.y = (minY + maxY) / 2f;
        Vector3 targetAreaSize = new Vector3(_currentLocalTargetRange, maxY - minY, _currentLocalTargetRange);
        Gizmos.DrawWireCube(targetAreaCenter, targetAreaSize);
    }
}

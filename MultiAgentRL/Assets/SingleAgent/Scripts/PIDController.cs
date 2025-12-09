using UnityEngine;

[System.Serializable] // 可以在 Inspector 中显示
public class PIDController
{
    public float Kp; // 比例增益
    public float Ki; // 积分增益
    public float Kd; // 微分增益

    [Tooltip("积分项的最大累积值。当误差持续存在时，防止 Ki 项无限增大导致积分饱和。")]
    public float integralLimit = 10f; // 新增：积分项限制，提供一个默认值
    [Tooltip("PID输出的最大值。限制控制器能施加的最大力矩或推力，防止过大的瞬时力导致失控。")]
    public float outputLimit = 100f;   // 新增：输出值限制，提供一个默认值
    
    // === 新增：用于控制是否打印调试信息 ===
    public bool enableDebugLog = false; // 默认不打印，可以在 Inspector 勾选
    public string debugName = "PID";   // 给 PID 起个名字，方便区分日志

    private float _integral;
    private float _previousError;
    private bool _initialized = false;

    // 清除控制器状态 (在每次剧集开始或重置时调用)
    public void Reset()
    {
        _integral = 0;
        _previousError = 0;
        _initialized = false;
        // Debug.Log($"[{debugName}] PID Controller Reset."); // 可以添加重置日志
    }

    // 计算控制输出
    // targetValue: 期望值
    // currentValue: 当前值
    // deltaTime: 时间步长 (FixedUpdate 的 Time.fixedDeltaTime)
    public float Calculate(float targetValue, float currentValue, float deltaTime)
    {
        // 避免除以零或在物理更新暂停时计算
        if (deltaTime == 0) return 0; 

        float error = targetValue - currentValue;

        if (!_initialized)
        {
            _previousError = error;
            _initialized = true;
        }

        // >>> 积分项计算并进行限制 <<<
        _integral += error * deltaTime; 
        // 使用 Mathf.Clamp 限制 _integral 的累积范围
        _integral = Mathf.Clamp(_integral, -integralLimit, integralLimit);

        // 微分项计算
        float derivative = (error - _previousError) / deltaTime; 

        _previousError = error; // 更新上一次误差

        // PID 计算核心
        float pTerm = Kp * error;
        float iTerm = Ki * _integral;
        float dTerm = Kd * derivative;

        float output = pTerm + iTerm + dTerm;

        // >>> 最终输出进行限制 <<<
        // 使用 Mathf.Clamp 限制最终输出的范围
        output = Mathf.Clamp(output, -outputLimit, outputLimit);

        // 调试信息：只在 enableDebugLog 为 true 时打印
        if (enableDebugLog)
        {
            Debug.Log($"[{debugName}] Error: {error:F3}, Integral (clamped): {_integral:F3}, Derivative: {derivative:F3}, Output (clamped): {output:F3}");
            Debug.Log($"[{debugName}] Target: {targetValue:F3}, Current: {currentValue:F3}");
        }

        return output;
    }
}
// 创建一个新的C#脚本，命名为 Dispatcher.cs
using System;
using System.Collections.Concurrent;
using UnityEngine;

public class Dispatcher : MonoBehaviour
{
    // 单例模式，方便从任何地方访问
    private static Dispatcher _instance;

    // 一个线程安全的队列，用于存放需要在主线程执行的委托（Action）
    private readonly ConcurrentQueue<Action> _executionQueue = new ConcurrentQueue<Action>();

    void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(this.gameObject); // 确保切换场景时它依然存在
        }
        else
        {
            Destroy(this.gameObject);
        }
    }

    void Update()
    {
        // 在主线程的每一帧，检查队列中是否有待办事项
        while (_executionQueue.TryDequeue(out var action))
        {
            // 如果有，就执行它
            action.Invoke();
        }
    }

    /// <summary>
    /// 从任何线程调用此方法，以请求在主线程上执行一个操作。
    /// </summary>
    /// <param name="action">要执行的操作</param>
    public static void RunOnMainThread(Action action)
    {
        if (_instance != null && action != null)
        {
            _instance._executionQueue.Enqueue(action);
        }
    }
}
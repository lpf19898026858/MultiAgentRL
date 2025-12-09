using UnityEngine;
using System.IO;

public class LoggingManager : MonoBehaviour
{
    public static LoggingManager Instance { get; private set; }
    private string logFilePath;

    void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject); 
            Initialize();
        }
        else
        {
            Destroy(gameObject);
        }
    }

    private void Initialize()
    {
        string logDirectory = Path.Combine(Application.dataPath, "..", "Logs");
        Directory.CreateDirectory(logDirectory);
        logFilePath = Path.Combine(logDirectory, $"TrainingLog_{System.DateTime.Now:yyyy-MM-dd_HH-mm-ss}.txt");
        File.WriteAllText(logFilePath, $"--- Unity ML-Agents Training Log Started at {System.DateTime.Now} ---\n\n");
        Debug.Log("LoggingManager Initialized. All logs will be written to: " + logFilePath);
    }

    void OnEnable()
    {
        Application.logMessageReceived += HandleLog;
    }

    void OnDisable()
    {
        Application.logMessageReceived -= HandleLog;
    }

    void HandleLog(string logString, string stackTrace, LogType type)
    {
        if (string.IsNullOrEmpty(logFilePath)) return;
        try
        {
            string formattedLog = $"[{System.DateTime.Now:HH:mm:ss}] [{type}] {logString}\n";
            if (type == LogType.Error || type == LogType.Exception)
            {
                formattedLog += stackTrace + "\n";
            }
            File.AppendAllText(logFilePath, formattedLog);
        }
        catch (IOException ex)
        {
            Debug.LogError($"Failed to write to log file: {ex.Message}");
        }
    }
}
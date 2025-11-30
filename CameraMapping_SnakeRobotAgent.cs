using System.Collections.Generic;
using System.IO;
using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.SceneManagement;

public class CameraMapping_SnakeRobotAgent : Agent
{
    TrailRenderer trailRenderer;

    public GameObject agentCamera; // ヘビ型ロボットのCamera（Transform用）
    public Camera sourceCamera;    // 追加：カメラ可視スキャナ用 Camera

    public GameObject Body1;
    public GameObject Body2;
    public GameObject Body3;

    Rigidbody rBody1;
    Rigidbody rBody2;
    Rigidbody rBody3;

    public List<MappingAxleInfo> MappingaxleInfos;
    public float maxMotorTorque;

    float stageNumber = 0.0f;

    int actionCount = 10;

    public int episodeCount1;
    public int episodeCount2;
    int[] collisionCount = new int[100];
    int fallCount = 0;
    int fallFlg = 0;
    int timeUpFlg = 1;

    public int episodeCountAll = 0;
    int actionStepCount = 0;
    int collisionCountLog = 0;

    private int newCellsSumThisEpisode = 0;
    private float distanceTravelled = 0f;
    private Vector3 prevPos;
    public float totalFloorArea = 25f;

    // カメラサンプリング数（ログ用）
    public int samplesX = 64;
    public int samplesY = 36;

    // 11/7　追加
    public int coverageReachedStepRaw = -1;                  // 90%到達時のステップ（actionStepCount）。到達しなければ-1 ログ用
    public int coverageStagnationCountAtReach = 0;           // 到達時点までの停滞ペナルティ回数のスナップショット　ログ用
    // 11/7　追加

    //private float areaReward = 0f;   // 11/05追加　このエピソードの面積報酬合計

    public int stagnationPenaltyCountThisEpisode = 0;
    public float coverageReportThreshold = 0.90f;
    private bool coverageReportedThisEpisode = false;

    public MarkerMeshPainter meshPainter;
    public float coverageThreshold = 0.9f;
    public float terminalRewardScale = 0.75f;

    public int stagnationWindowSteps = 50;
    public int stagnationMinNewCells = 1;
    public float stagnationPenalty = -0.01f;
    public bool endEpisodeOnStagnation = false;
    private Queue<int> recentNewCells = new Queue<int>();
    private int recentNewCellsSum = 0;

    public int obsWindowSteps = 25;
    private Queue<int> obsRecentNewCells = new Queue<int>();
    private int obsRecentNewCellsSum = 0;
    public int maxNewCellsPerStepEstimate = 50;

    public int noNewCellCountThisEpisode = 0;

    public float targetFoundReward = 1.0f;

    public MeshMarkerCombiner combiner; // 任意参照（未使用ならnullでOK）

    bool evalFlg = true;
    float episodeReward = 0.0f;

    public event System.Action OnEndEpisode;
    public event System.Action OnStartEpisode;

    // カメラスキャナ
    public CameraScannerMapping camScanner;
    public float areaRewardScale = 0.05f; //差分の場合
    //public float areaRewardScale = 0.0001f; //総面積の場合

    public TargetManager targetManager;

    public override void Initialize()
    {
        rBody1 = Body1.GetComponent<Rigidbody>();
        rBody2 = Body2.GetComponent<Rigidbody>();
        rBody3 = Body3.GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // 直前エピソードのログ出力
        Logger();

        //Debug.Log(areaReward); // 11/05　面積報酬表示
        //areaReward = 0; // 面積報酬リセット
        //Debug.Log(episodeReward); // 11/13報酬表示

        // 11/7　追加
        coverageReportedThisEpisode = false;       // 90%到達フラグをリセット
        coverageReachedStepRaw = -1;               // 到達ステップをリセット
        coverageStagnationCountAtReach = 0;        // 到達時の停滞回数をリセット
        // 11/7　追加

        fallFlg = 0; // 11/7追加　落下フラグの初期化

        actionStepCount = 0;
        timeUpFlg = 1;
        collisionCountLog = 0;
        episodeReward = 0.0f;
        episodeCountAll += 1;

        noNewCellCountThisEpisode = 0;
        stagnationPenaltyCountThisEpisode = 0;
        coverageReportedThisEpisode = false;

        if (meshPainter != null) meshPainter.ClearAll();

        if (targetManager != null)
            targetManager.ResetAndPlaceRandom();

        stageNumber = Academy.Instance.EnvironmentParameters.GetWithDefault("stage_number", 0.0f);
        if (stageNumber > 1.0f) { Application.Quit(); }

        newCellsSumThisEpisode = 0;
        distanceTravelled = 0f;
        prevPos = agentCamera.transform.position;

        rBody1.angularVelocity = Vector3.zero;
        rBody1.velocity = Vector3.zero;
        rBody2.angularVelocity = Vector3.zero;
        rBody3.velocity = Vector3.zero;
        rBody3.angularVelocity = Vector3.zero;
        rBody3.velocity = Vector3.zero;

        Body1.transform.position = new Vector3(0.0f, 0.1f, 0.1f);
        Body1.transform.rotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);
        Body2.transform.position = new Vector3(0.0f, 0.1f, 0.1f);
        Body2.transform.rotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);
        Body3.transform.position = new Vector3(0.0f, 0.1f, 0.1f);
        Body3.transform.rotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);

        trailRenderer = GetComponent<TrailRenderer>();
        if (trailRenderer != null) trailRenderer.Clear();

        OnStartEpisode?.Invoke();

        recentNewCells.Clear();
        recentNewCellsSum = 0;
        obsRecentNewCells.Clear();
        obsRecentNewCellsSum = 0;
    }

    public void EndThisEpisode()
    {
        EndEpisode();
        OnEndEpisode?.Invoke();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(agentCamera.transform.position.x);
        sensor.AddObservation(agentCamera.transform.position.z);
        sensor.AddObservation(rBody1.velocity.x);
        sensor.AddObservation(rBody1.velocity.z);
        sensor.AddObservation(rBody2.velocity.x);
        sensor.AddObservation(rBody2.velocity.z);
        sensor.AddObservation(rBody3.velocity.x);
        sensor.AddObservation(rBody3.velocity.z);
        sensor.AddObservation(Body1.transform.eulerAngles.y);
        sensor.AddObservation(Body2.transform.eulerAngles.y);
        sensor.AddObservation(Body3.transform.eulerAngles.y);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        actionStepCount += 1;
        actionCount -= 1;

        // 既存の車輪制御はそのまま
        Vector3 firstLeftWheelControlSignal = Vector3.zero;
        Vector3 firstRightWheelControlSignal = Vector3.zero;
        Vector3 secondLeftWheelControlSignal = Vector3.zero;
        Vector3 secondRightWheelControlSignal = Vector3.zero;
        Vector3 thirdLeftWheelControlSignal = Vector3.zero;
        Vector3 thirdRightWheelControlSignal = Vector3.zero;

        MappingaxleInfos[0].firstLeftWheel.brakeTorque = 0f;
        MappingaxleInfos[0].firstRightWheel.brakeTorque = 0f;
        MappingaxleInfos[0].secondLeftWheel.brakeTorque = 0f;
        MappingaxleInfos[0].secondRightWheel.brakeTorque = 0f;
        MappingaxleInfos[0].thirdLeftWheel.brakeTorque = 0f;
        MappingaxleInfos[0].thirdRightWheel.brakeTorque = 0f;

        int firstLeftWheelAction = actionBuffers.DiscreteActions[0];
        int firstRightWheelAction = actionBuffers.DiscreteActions[1];
        int secondLeftWheelAction = actionBuffers.DiscreteActions[2];
        int secondRightWheelAction = actionBuffers.DiscreteActions[3];
        int thirdLeftWheelAction = actionBuffers.DiscreteActions[4];
        int thirdRightWheelAction = actionBuffers.DiscreteActions[5];

        if (firstLeftWheelAction == 0) firstLeftWheelControlSignal.z = 1.0f;
        else if (firstLeftWheelAction == 1) firstLeftWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].firstLeftWheel.brakeTorque = 100f;

        if (firstRightWheelAction == 0) firstRightWheelControlSignal.z = 1.0f;
        else if (firstRightWheelAction == 1) firstRightWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].firstRightWheel.brakeTorque = 100f;

        if (secondLeftWheelAction == 0) secondLeftWheelControlSignal.z = 1.0f;
        else if (secondLeftWheelAction == 1) secondLeftWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].secondLeftWheel.brakeTorque = 100f;

        if (secondRightWheelAction == 0) secondRightWheelControlSignal.z = 1.0f;
        else if (secondRightWheelAction == 1) secondRightWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].secondRightWheel.brakeTorque = 100f;

        if (thirdLeftWheelAction == 0) thirdLeftWheelControlSignal.z = 1.0f;
        else if (thirdLeftWheelAction == 1) thirdLeftWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].thirdLeftWheel.brakeTorque = 100f;

        if (thirdRightWheelAction == 0) thirdRightWheelControlSignal.z = 1.0f;
        else if (thirdRightWheelAction == 1) thirdRightWheelControlSignal.z = -1.0f;
        else MappingaxleInfos[0].thirdRightWheel.brakeTorque = 100f;

        float[] motor = new float[6];
        motor[0] = maxMotorTorque * firstLeftWheelControlSignal.z;
        motor[1] = maxMotorTorque * firstRightWheelControlSignal.z;
        motor[2] = maxMotorTorque * secondLeftWheelControlSignal.z;
        motor[3] = maxMotorTorque * secondRightWheelControlSignal.z;
        motor[4] = maxMotorTorque * thirdLeftWheelControlSignal.z;
        motor[5] = maxMotorTorque * thirdRightWheelControlSignal.z;

        MappingaxleInfos[0].firstLeftWheel.motorTorque = motor[0];
        MappingaxleInfos[0].firstRightWheel.motorTorque = motor[1];
        MappingaxleInfos[0].secondLeftWheel.motorTorque = motor[2];
        MappingaxleInfos[0].secondRightWheel.motorTorque = motor[3];
        MappingaxleInfos[0].thirdLeftWheel.motorTorque = motor[4];
        MappingaxleInfos[0].thirdRightWheel.motorTorque = motor[5];

        camScanner.StepScan();

        // カメラスキャナによる新規セル数
        int newCellsThisStep = 0;
        if (camScanner != null)
        {
            newCellsThisStep = camScanner.GetAndResetNewMarksThisStep();

            // ターゲットを見つけた瞬間の報酬（必要なら）
            if (camScanner.GetAndResetTargetHitThisStep())
            {
                //AddReward(targetFoundReward);
                //episodeReward += targetFoundReward;
            }
        }

        // 11/13追加　総面積報酬
        //if (meshPainter != null)
        //{
        //    // 現時点で塗られている全セル面積
        //    float totalMarkedArea = meshPainter.GetMarkedArea(); // = markedCells * cellSize^2
        //    float reward = totalMarkedArea * areaRewardScale;
        //    AddReward(reward);
        //    episodeReward += reward;            
        //}

        // 11/13追加　停滞したステップ数のカウント
        //if (newCellsThisStep == 0)
        //{
        //    noNewCellCountThisEpisode++;
        //}

        //増分の面積報酬
        if (newCellsThisStep > 0 && meshPainter != null)
        {
            float cellArea = meshPainter.cellSize * meshPainter.cellSize;
            float reward = newCellsThisStep * cellArea * areaRewardScale;
            AddReward(reward);
            episodeReward += reward;
        }
        else
        {
            noNewCellCountThisEpisode++;
        }        

        // 停滞判定ウィンドウ更新
        recentNewCells.Enqueue(newCellsThisStep);
        recentNewCellsSum += newCellsThisStep;
        if (recentNewCells.Count > stagnationWindowSteps)
            recentNewCellsSum -= recentNewCells.Dequeue();

        if (recentNewCells.Count == stagnationWindowSteps && recentNewCellsSum < stagnationMinNewCells)
        {
            AddReward(stagnationPenalty);
            episodeReward += stagnationPenalty;
            stagnationPenaltyCountThisEpisode++;

            if (endEpisodeOnStagnation)
            {
                timeUpFlg = 0;
                EndThisEpisode();
                return;
            }
            recentNewCells.Clear();
            recentNewCellsSum = 0;
        }

        obsRecentNewCells.Enqueue(newCellsThisStep);
        obsRecentNewCellsSum += newCellsThisStep;
        if (obsRecentNewCells.Count > obsWindowSteps)
            obsRecentNewCellsSum -= obsRecentNewCells.Dequeue();

        // 探索率（Coverage）
        float coverageRate = 0f;
        if (meshPainter != null && totalFloorArea > 0f)
        {
            int markedCells = meshPainter.GetMarkedCellCount();
            float cellArea = meshPainter.cellSize * meshPainter.cellSize;
            float markedArea = markedCells * cellArea;
            coverageRate = markedArea / totalFloorArea;
        }

        // 90%超えた瞬間に一度だけログ
        if (!coverageReportedThisEpisode && coverageRate >= coverageReportThreshold)
        {
            coverageReportedThisEpisode = true;
            coverageReachedStepRaw = actionStepCount;                 // 生のステップ数（後で×5してログに出す）
            coverageStagnationCountAtReach = stagnationPenaltyCountThisEpisode;

            Debug.Log($"[Episode {episodeCountAll}] Coverage {coverageRate * 100f:F1}% | Step {actionStepCount} | Stagnation penalties: {stagnationPenaltyCountThisEpisode}");
        }

        // 11/13追加　探索率100％に到達したときに早期終了
        float earlyDoneThreshold = 0.999f; // 誤差吸収。厳密に100%なら 1.0f でもOK
        if (coverageRate >= earlyDoneThreshold)
        {
            // 達成ボーナス
            // AddReward(terminalRewardScale); // 例: 0.5〜2.0

            timeUpFlg = 0; // あなたのログ用フラグ運用に合わせて
            Debug.Log($"[Episode {episodeCountAll}] Coverage reached 100% -> Early end at step {actionStepCount}");
            EndThisEpisode(); // あなたのラッパーで EndEpisode と OnEndEpisode を呼ぶ
            return;
        }
        // 11/13追加

        newCellsSumThisEpisode += newCellsThisStep;

        // 走行距離（XZ）
        Vector3 currPos = agentCamera.transform.position;
        distanceTravelled += Vector3.Distance(prevPos, currPos);
        prevPos = currPos;

        // 落下判定
        if (Body1.transform.position.y < 0 && Body2.transform.position.y < 0 && Body3.transform.position.y < 0)
        {
            AddReward(-0.5f);
            episodeReward += -0.05f;//ログ用
            fallFlg = 1;
            timeUpFlg = 0;
            fallCount += 1;
            EndThisEpisode();
        }
    }

    public void CollisionChecker(Collision collision)
    {
        if (collision.gameObject.name.Contains("Cube"))
        {
            collisionCountLog += 1;

            int idx = episodeCount2 - episodeCount1;
            if (idx >= 0 && idx < collisionCount.Length)
                collisionCount[idx] += 1;

            episodeReward += -0.05f;
            AddReward(-0.05f);
        }
    }

    public override void Heuristic(in ActionBuffers actionBuffers)
    {
        var actionsOut = actionBuffers.DiscreteActions;
        actionsOut[0] = 0;
        actionsOut[1] = 0;
        actionsOut[2] = 0;
        actionsOut[3] = 0;
        actionsOut[4] = 0;
        actionsOut[5] = 0;

        if (Input.GetKey(KeyCode.Alpha2)) actionsOut[0] = 0;
        if (Input.GetKey(KeyCode.Alpha3)) actionsOut[1] = 0;
        if (Input.GetKey(KeyCode.Alpha4)) actionsOut[2] = 0;
        if (Input.GetKey(KeyCode.Alpha5)) actionsOut[3] = 0;
        if (Input.GetKey(KeyCode.Alpha6)) actionsOut[4] = 0;
        if (Input.GetKey(KeyCode.Alpha7)) actionsOut[5] = 0;
    }

    // 既存Loggerを最小変更で流用（設定行だけカメラ用に）
    private static string logFilePath = null;
    private static List<string> logBuffer = new List<string>();
    private static readonly object logFileLock = new object();
    private const int BufferLimit = 5;
    private static bool isLogFilePathInitialized = false;
    public string logBaseDir = @"D:\LAB_Drive\Data\KazuyaMaruyama\Logs";

    public void Logger()
    {
        if (!isLogFilePathInitialized)
        {
            if (!Directory.Exists(logBaseDir))
                Directory.CreateDirectory(logBaseDir);

            logFilePath = GetNextAvailableFileNameInDir(logBaseDir, "TrainLog_Cam", ".csv");
            isLogFilePathInitialized = true;

            float cellSizeForConfig = (meshPainter != null) ? meshPainter.cellSize : 0f;

            bool newFile = !File.Exists(logFilePath);
            if (newFile)
            {
                // 水平FOVの近似（verticalFOVとaspectから算出）
                float vFov = (sourceCamera != null) ? sourceCamera.fieldOfView : 0f;
                float aspect = (sourceCamera != null) ? sourceCamera.aspect : 1f;
                float hFov = 2f * Mathf.Atan(Mathf.Tan(vFov * Mathf.Deg2Rad / 2f) * aspect) * Mathf.Rad2Deg;

                string config = $"Config,cellSize,{cellSizeForConfig},samplesX,{samplesX},samplesY,{samplesY},hFOV,{hFov:F1},vFOV,{vFov:F1}";
                File.AppendAllText(logFilePath, config + Environment.NewLine);

                string header = "EpisodeIndex,Steps,MarkedCells,MarkedArea,CoverageRate,StagnationCount,NoNewCellSteps,Reward,Collisions,Fell,Timeout,DistanceTravelled,NewCellsPerStepAvg,Coverage90Step,Coverage90StagCount";
                File.AppendAllText(logFilePath, header + Environment.NewLine);
            }
        }

        try
        {
            if (episodeCountAll > 0)
            {
                int markedCells = (meshPainter != null) ? meshPainter.GetMarkedCellCount() : 0;
                float cellSizeLocal = (meshPainter != null) ? meshPainter.cellSize : 0f;
                float markedArea = markedCells * cellSizeLocal * cellSizeLocal;
                float coverageRate = (totalFloorArea > 0f) ? (markedArea / totalFloorArea) : 0f;
                float efficiencyAreaPerStep = (actionStepCount > 0) ? (markedArea / actionStepCount) : 0f;
                float newCellsPerStepAvg = (actionStepCount > 0) ? ((float)newCellsSumThisEpisode / actionStepCount) : 0f;

                int coverage90StepForLog = (coverageReachedStepRaw >= 0) ? (coverageReachedStepRaw * 5) : -1; // Stepsと同じ係数×5に揃える
                int coverage90StagCountForLog = coverageStagnationCountAtReach;

                string data = string.Format(
                "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14}",
                episodeCountAll,
                actionStepCount * 5,
                markedCells,
                markedArea,
                coverageRate,
                stagnationPenaltyCountThisEpisode,
                noNewCellCountThisEpisode,
                episodeReward,
                collisionCountLog,
                fallFlg,
                timeUpFlg,
                distanceTravelled,
                newCellsPerStepAvg,
                coverage90StepForLog,         // 追加1
                coverage90StagCountForLog     // 追加2
                );

                lock (logFileLock)
                {
                    logBuffer.Add(data);
                    if (logBuffer.Count >= BufferLimit)
                    {
                        File.AppendAllLines(logFilePath, logBuffer);
                        logBuffer.Clear();
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"ログ書き込みエラー: {ex.Message}");
        }
    }

    private void OnApplicationQuit()
    {
        lock (logFileLock)
        {
            if (logBuffer.Count > 0)
            {
                File.AppendAllLines(logFilePath, logBuffer);
                logBuffer.Clear();
            }
        }
    }

    private static string GetNextAvailableFileNameInDir(string baseDir, string baseFileName, string fileExtension)
    {
        int fileIndex = 1;
        string path;
        do
        {
            string fileName = $"{baseFileName}{fileIndex}{fileExtension}";
            path = Path.Combine(baseDir, fileName);
            fileIndex++;
        } while (File.Exists(path));
        return path;
    }
}

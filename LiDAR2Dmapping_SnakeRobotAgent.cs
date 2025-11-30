using System.Collections;
using System.Collections.Generic;
// 6/25追加　ログで使用
using System;
using System.IO;
using System.Text;
// 6/25追加
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine.SceneManagement;
using Unity.VisualScripting;

/*
    CubeとPlaneの1辺の長さ(初期値)は
    Cube:1m
    Plane:10m
*/

[System.Serializable]
public class TwoDMappingAxleInfo
{
    public WheelCollider firstLeftWheel;
    public WheelCollider firstRightWheel;
    public WheelCollider secondLeftWheel;
    public WheelCollider secondRightWheel;
    public WheelCollider thirdLeftWheel;
    public WheelCollider thirdRightWheel;
}

// SnakeRobotAgent
public class LiDAR2Dmapping_SnakeRobotAgent : Agent
{
    public GameObject agentCamera; //ヘビ型ロボットのCamera

    public GameObject Body1; // Body1のGameObject
    public GameObject Body2; // Body2のGameObject
    public GameObject Body3; // Body3のGameObject

    Rigidbody rBody1; // Body1のRigidbody
    Rigidbody rBody2; // Body2のRigidbody
    Rigidbody rBody3; // Body3のRigidbody

    public List<TwoDMappingAxleInfo> TwoDMappingaxleInfos;
    public float maxMotorTorque; // 最大モータートルク

    float stageNumber = 0.0f; // エピソード開始時のステージ番号を取得

    int actionCount = 10; // 行動の回数をカウント

    public CameraScannerMapping camScanner; // 11/05追加
    public bool useCameraForPainting = true;

    // episodeCount1とepisodeCount2は同じ値にすること
    public int episodeCount1; // 指定したエピソード回数をカウント（評価用）
    public int episodeCount2; // ゴール到達率の分母（評価用）
    int[] collisionCount = new int[100]; // ゴール到達までに瓦礫に衝突した回数（1エピソードごとに算出）
    int fallCount = 0; // フィールドから落下した回数をカウント（評価用）
    int fallFlg = 0; // フィールドから落下（1なら落下0なら未落下）
    int timeUpFlg = 1; // 時間切れ（1なら時間切れ0なら時間内）

    // 6/18追加　ログで使用
    public int episodeCountAll = 0; // 学習完了までのエピソード数をカウント
    int actionStepCount = 0; // 行動の回数をカウント
    int collisionCountLog = 0; // ゴール到達までに瓦礫に衝突した回数（1エピソードごとに算出）Log用
    // 6/18追加

    // 9/29 追加: 床マーキングログ用の集計フィールド Quda
    private int newCellsSumThisEpisode = 0;      // 今エピソードで新規に塗ったセル総数
    private float distanceTravelled = 0f;        // 走行距離
    private Vector3 prevPos;                     // 前フレーム位置
    public float totalFloorArea = 25f;           // 探索可能な床面積（任意に設定、例: 25m^2）
    public int numberOfRaysH = 180;              // 水平本数 ログ用
    private int markedCellsCache;
    private float markedAreaCache;
    private float coverageRateCache;
    private float efficiencyAreaPerStepCache;
    private float newCellsPerStepAvgCache;
    public int numberOfRaysV = 1;                // 垂直本数。水平のみなら1 ログ用
    public float hFOV = 180f;                    // 水平視野角 ログ用
    public float vFOV = 0f;                      // 垂直視野角。水平のみなら0 ログ用
    // 9/27追加

    // 11/7　追加
    public int coverageReachedStepRaw = -1;                  // 90%到達時のステップ（actionStepCount）。到達しなければ-1 ログ用
    public int coverageStagnationCountAtReach = 0;           // 到達時点までの停滞ペナルティ回数のスナップショット　ログ用
    // 11/7　追加

    public int stagnationPenaltyCountThisEpisode = 0; // 10/9追加　このエピソードで停滞ペナルティが入った回数
    public float coverageReportThreshold = 0.90f; // 探索率90%でレポート
    private bool coverageReportedThisEpisode = false; // このエピソードで報告済みか

    // 10/1追加
    public MarkerMeshPainter meshPainter;      // インスペクタで MarkerMeshRoot を割り当て
    public float coverageThreshold = 0.9f;
    public float terminalRewardScale = 0.75f;
    // 10/1追加

    // 停滞判定用の設定 10/9追加
    public int stagnationWindowSteps = 50;      // 直近50ステップで判定
    public int stagnationMinNewCells = 1;       // 50ステップで最低1セルは増えてほしい
    public float stagnationPenalty = -0.01f;     // 停滞ペナルティ
    public bool endEpisodeOnStagnation = false; // trueなら停滞で終了
    // 内部状態
    private Queue<int> recentNewCells = new Queue<int>();
    private int recentNewCellsSum = 0;
    public int obsWindowSteps = 25; // 観察用の窓幅
    private Queue<int> obsRecentNewCells = new Queue<int>();
    private int obsRecentNewCellsSum = 0;
    // 上限正規化用：1ステップの理論最大新規セル数（仮にLiDARのペイントが最大Mセル）
    public int maxNewCellsPerStepEstimate = 50; // チューニング必須
    // 10/9追加

    public int noNewCellCountThisEpisode = 0; // 10/30追加 探索済みエリアが増えなかったステップ数

    public float targetFoundReward = 1.0f; // 10/14追加　ターゲット発見報酬

    public MeshMarkerCombiner combiner; // 統合メッシュの参照 9/30追加

    bool evalFlg = true; // trueは未評価、falseは評価済み
    float episodeReward = 0.0f; // 1エピソードごとの獲得報酬を計測

    //追加1
    public event System.Action OnEndEpisode;
    public event System.Action OnStartEpisode;
    //ここまで

    public LiDARDownwardFloorPainter downPainter; // 11/04　追加

    // 9/26追加 Quda
    public float areaRewardScale = 0.05f; // 面積報酬係数（調整用）
    // 9/26追加

    public TargetManager targetManager; // 10/14追加 Inspectorで割り当て

    public LiDARScanner2DMapping lidar;// 11/11

    // 11/10追加
    public int localPatchCells = 21; // 奇数推奨（中心セルが明確）
    public bool useLidarGridObservation = true;
    // 11/10追加

    //public bool debugPrintPatch = true; // 11/20 デバック出力ON/OFF

    public TrailRenderer agentTrail;// 11/11追加

    public OccupancyGridMiniMap miniMapController; // 11/13追加 UIミニマップの制御スクリプト参照

    public ObstacleManager obstacleManager; //　11/21追加　オブジェクトランダム配置

    // 11/20追加　マップ　2次元配列デバック
    //private void DebugPrintLocalPatch(float[] patch, int size, string title = "Patch")
    //{
    //    if (!debugPrintPatch || patch == null || patch.Length != size * size) return;

    //    System.Text.StringBuilder sb = new System.Text.StringBuilder();
    //    sb.AppendLine($"[{title}] size={size}x{size} (0=free, 0.5=unknown, 1=occupied)");

    //    for (int z = size - 1; z >= 0; z--) // 上（前方）を先に出したければ逆順
    //    {
    //        for (int x = 0; x < size; x++)
    //        {
    //            float v = patch[z * size + x];
    //            sb.Append(v.ToString("0.00")).Append(' ');
    //        }
    //        sb.AppendLine();
    //    }
    //    Debug.Log(sb.ToString());
    //}
    // 11/20追加

    // ゲームオブジェクト生成時に呼ばれる
    public override void Initialize()
    {
        // BodyのRigidBodyの参照の取得
        rBody1 = Body1.GetComponent<Rigidbody>();
        rBody2 = Body2.GetComponent<Rigidbody>();
        rBody3 = Body3.GetComponent<Rigidbody>();
        
    }

    // エピソード開始時に呼ばれる
    public override void OnEpisodeBegin()
    {
        /*// 前エピソードの表示をクリア 9/30追加 Quda
        var mm = lidar?.markerManager;
        if (mm != null) mm.ClearAll();

        if (combiner != null) combiner.ClearCombinedMesh();
        // 9/30追加 */

        // 集計値の取得
        //int markedCells = lidar != null && lidar.markerManager != null ? lidar.markerManager.GetMarkedCellCount() : 0; //Quda
        //float cellSizeSq = lidar != null && lidar.markerManager != null ? (lidar.markerManager.cellSize * lidar.markerManager.cellSize) : 0f; //Quda
        int markedCells = lidar != null && lidar.meshPainter != null ? lidar.meshPainter.GetMarkedCellCount() : 0; // 10/1
        float cellSizeSq = lidar != null && lidar.meshPainter != null ? (lidar.meshPainter.cellSize * lidar.meshPainter.cellSize) : 0f; //10/1
        float markedArea = markedCells * cellSizeSq;
        //float coverageRate = (totalFloorArea > 0f) ? (markedArea / totalFloorArea) : 0f;
        float obstacleArea = (obstacleManager != null) ? obstacleManager.GetTotalObstacleFootprintArea() : 0f; //　11/21追加
        float effectiveFloorArea = Mathf.Max(1e-6f, totalFloorArea - obstacleArea); //　11/21追加 分母補正
        float coverageRate = markedArea / effectiveFloorArea; //　11/21追加障害物あり
        float efficiencyAreaPerStep = (actionStepCount > 0) ? (markedArea / actionStepCount) : 0f;
        float newCellsPerStepAvg = (actionStepCount > 0) ? ((float)newCellsSumThisEpisode / actionStepCount) : 0f;

        //Debug.Log($"[Episode {episodeCountAll}] Coverage {coverageRate * 100f:F1}% | Stagnation penalties so far: {stagnationPenaltyCountThisEpisode}");//　10/24追加

        // 6/18追加　ログで使用
        Logger(); // 1つ前のエピソードの結果を記録
        actionStepCount = 0; // エピソード開始時に行動の回数を初期化
        timeUpFlg = 1; // timeUpFlgを初期化
        collisionCountLog = 0; // collisionCountLogを初期化
        episodeReward = 0.0f; // episodeRewardを初期化
        episodeCountAll += 1; // エピソード回数を+1
                              // Debug.Log("1:" + actionStepCount);
                              // Debug.Log("MaxStep:" + MaxStep);
                              // 6/18追加

        obstacleManager.ResetRandom();//　11/21障害物の位置リセット

        // 11/7　追加
        coverageReportedThisEpisode = false;       // 90%到達フラグをリセット
        coverageReachedStepRaw = -1;               // 到達ステップをリセット
        coverageStagnationCountAtReach = 0;        // 到達時の停滞回数をリセット
        // 11/7　追加

        fallFlg = 0; // 11/7追加　落下フラグの初期化

        noNewCellCountThisEpisode = 0; // 10/30追加　探索済みエリアが増えなかったステップ数のカウントリセット

        // 11/5追加
        // ペインターをクリア（両シーン共通）
        if (meshPainter != null) meshPainter.ClearAll();

        // カメラスキャナの手動ステップを有効化（Agentから呼ぶ）
        if (camScanner != null) camScanner.manualStep = true;
        // 11/5追加

        // 10/1追加
        // 前エピソードの表示をクリア
        if (meshPainter != null) meshPainter.ClearAll();
        // 10/1追加

        stagnationPenaltyCountThisEpisode = 0; //　停滞ペナルティの回数をリセット
        coverageReportedThisEpisode = false; // レポートフラグをリセット

        // 10/14追加　ターゲットをランダム配置＆発見状態リセット
        if (targetManager != null)
            targetManager.ResetAndPlaceRandom();

        // 6/25追加　早期終了
        // エピソード開始時のステージ番号を取得
        stageNumber = Academy.Instance.EnvironmentParameters.GetWithDefault("stage_number", 0.0f);

        //"box"はunityのシーンの名前を入れる、stageNumberは.yamlのvalueの値となる    
        // 現在のステージの学習が終わったらアプリケーションを終了
        if (stageNumber > 1.0f)
        {
            Application.Quit();
        }
        // 6/25追加

        // 9/29追加 マーカークリア
        newCellsSumThisEpisode = 0;
        distanceTravelled = 0f;
        prevPos = agentCamera.transform.position;
        // 9/29追加

        // 11/11追加
        if (agentTrail != null)
        {
            // 一旦無効化して完全に消す
            agentTrail.emitting = false;
            agentTrail.enabled = false;
            agentTrail.Clear();

            // 位置を初期化してから再有効化
            agentTrail.enabled = true;
            agentTrail.emitting = true;
        }
        // 11/11追加

        // 11/13追加
        // 占有グリッドのクリア
        if (lidar != null)
            lidar.ClearGrid();

        // RawImageのミニマップを使っているなら、テクスチャもクリア（灰色や透明で塗り直し）
        if (miniMapController != null)
            miniMapController.ClearTexture(); // 下で実装例

        // MarkerMeshPainterで床に可視化しているならクリア
        if (meshPainter != null)
            meshPainter.ClearAll();

        // 2DマップとTrail、その他が確実に消えるように
        OnStartEpisode?.Invoke();
        // 11/13追加

        // エピソード開始時のステージ番号を取得
        stageNumber = Academy.Instance.EnvironmentParameters.GetWithDefault("stage_number", 0.0f);

        // 現在のステージの学習が終わったらアプリケーションを終了
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage1" && stageNumber > 1.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage2" && stageNumber > 2.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage3" && stageNumber > 3.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage4" && stageNumber > 4.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage5" && stageNumber > 5.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage6" && stageNumber > 6.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage7" && stageNumber > 7.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage8" && stageNumber > 8.0f)
        {
            Application.Quit();
        }
        if (SceneManager.GetActiveScene().name == "SnakeRobot-Stage9" && stageNumber > 9.0f)
        {
            Application.Quit();
        }

        // SnakeRobotAgentの位置と速度をリセット
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

        //追加1
        OnStartEpisode?.Invoke();
        //ここまで

        // 10/9追加
        recentNewCells.Clear();
        recentNewCellsSum = 0;
        obsRecentNewCells.Clear();
        obsRecentNewCellsSum = 0;
        // 10/9追加
    }


    //追加1
    public void EndThisEpisode()
    {
        EndEpisode();
        OnEndEpisode?.Invoke();
    }
    //ここまで


    // 観察取得時に呼ばれる
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(agentCamera.transform.position.x); //SnakeRobotAgentのX座標
        sensor.AddObservation(agentCamera.transform.position.z); //SnakeRobotAgentのZ座標
        sensor.AddObservation(rBody1.velocity.x); //Body1のX速度
        sensor.AddObservation(rBody1.velocity.z); //Body1のZ速度
        sensor.AddObservation(rBody2.velocity.x); //Body2のX速度
        sensor.AddObservation(rBody2.velocity.z); //Body2のZ速度
        sensor.AddObservation(rBody3.velocity.x); //Body3のX速度
        sensor.AddObservation(rBody3.velocity.z); //Body3のZ速度
        sensor.AddObservation(Body1.transform.eulerAngles.y); //Body1のY角度
        sensor.AddObservation(Body2.transform.eulerAngles.y); //Body2のY角度
        sensor.AddObservation(Body3.transform.eulerAngles.y); //Body3のY角度

        if (lidar != null) // 11/6追加　最も近い障害物の距離＋方向の観察
        {
            var polar = lidar.GetNearestObstacleObservation();
            sensor.AddObservation(polar.distNorm);
            sensor.AddObservation(polar.angleNorm);
        }

        // 追加：占有グリッドのローカルパッチ
        if (useLidarGridObservation && lidar != null && lidar.occupancyGrid != null)
        {
            int w, h;
            Vector3 agentPos = agentCamera.transform.position;  // 基準位置（カメラでもBody1でも可）
            Vector3 agentForward = Body1.transform.forward;      // 進行方向（代表ボディ）
            float[] patch = lidar.occupancyGrid.GetEgocentricPatch(agentPos, agentForward, localPatchCells, out w, out h);

            // ベクトル観察として追加（0..1：free～occupied。未知は0.5）
            for (int i = 0; i < patch.Length; i++)
                sensor.AddObservation(patch[i]);
        }

        // 11/10追加：LiDAR占有グリッドパッチ
        if (useLidarGridObservation && lidar != null && lidar.occupancyGrid != null)
        {
            int w, h; 
            var agentPos = agentCamera.transform.position;
            var agentForward = Body1.transform.forward;
            var patch = lidar.occupancyGrid.GetEgocentricPatch(agentPos, agentForward, localPatchCells, out w, out h);
            for (int i = 0; i < patch.Length; i++) sensor.AddObservation(patch[i]);

            // 11/20追加 観察に追加する前や後でデバッグ表示
            //DebugPrintLocalPatch(patch, localPatchCells, "EgocentricPatch");
            // 11/20追加
        }
        // 11/10追加
    }

    // 直近の新規セル合計を正規化して追加 10/9追加
    //float normRecentNew = 0f;
    //int denom = Mathf.Max(1, obsWindowSteps * Mathf.Max(1, maxNewCellsPerStepEstimate));
    //normRecentNew = Mathf.Clamp01((float)obsRecentNewCellsSum / denom);
    //sensor.AddObservation(normRecentNew);
    //// 進行方向（XZ）
    //Vector3 fwd = Body1.transform.forward; // 代表ボディ
    //Vector2 fwd2 = new Vector2(fwd.x, fwd.z);
    //if (fwd2.sqrMagnitude < 1e-6f) fwd2 = Vector2.right; // 万が一ゼロなら右向きに仮置き
    //fwd2.Normalize();

    //// 未到達方向（重心に向かう方向）
    //Vector2 unvisitedDir2 = Vector2.zero;
    //if (meshPainter != null)
    //{
    //    Vector2 centroid = meshPainter.GetUnvisitedCentroidXZ(); // 重心座標（XZ）
    //    Vector2 agentXZ = new Vector2(agentCamera.transform.position.x, agentCamera.transform.position.z);
    //    Vector2 toCentroid = centroid - agentXZ;
    //    if (toCentroid.sqrMagnitude >= 1e-8f)
    //    {
    //        unvisitedDir2 = toCentroid.normalized;
    //    }
    //    else
    //    {
    //        unvisitedDir2 = Vector2.zero; // 重心と同位置ならゼロ
    //    }
    //}

    //// 角度差計算（符号付き −180..180 → −1..1へ正規化）
    //float normSignedAngle = 0f;
    //if (unvisitedDir2.sqrMagnitude >= 1e-8f)
    //{
    //    float signedAngle = Vector2.SignedAngle(fwd2, unvisitedDir2); // −180..180
    //    normSignedAngle = signedAngle / 180f; // −1..1
    //}
    //// 観察に追加
    //sensor.AddObservation(normSignedAngle);

    // 行動決定時に呼ばれる
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // 6/18追加 ログで使用
        actionStepCount = actionStepCount + 1; // 行動の回数をカウント
        // 6/18追加

        actionCount -= 1;

        // SnakeRobotに力を加える
        Vector3 firstLeftWheelControlSignal = Vector3.zero;
        Vector3 firstRightWheelControlSignal = Vector3.zero;
        Vector3 secondLeftWheelControlSignal = Vector3.zero;
        Vector3 secondRightWheelControlSignal = Vector3.zero;
        Vector3 thirdLeftWheelControlSignal = Vector3.zero;
        Vector3 thirdRightWheelControlSignal = Vector3.zero;

        // ブレーキを解除
        TwoDMappingaxleInfos[0].firstLeftWheel.brakeTorque = 0f;
        TwoDMappingaxleInfos[0].firstRightWheel.brakeTorque = 0f;
        TwoDMappingaxleInfos[0].secondLeftWheel.brakeTorque = 0f;
        TwoDMappingaxleInfos[0].secondRightWheel.brakeTorque = 0f;
        TwoDMappingaxleInfos[0].thirdLeftWheel.brakeTorque = 0f;
        TwoDMappingaxleInfos[0].thirdRightWheel.brakeTorque = 0f;

        // Discreteの場合
        // Agentが選択した行動を取得
        int firstLeftWheelAction = actionBuffers.DiscreteActions[0];
        int firstRightWheelAction = actionBuffers.DiscreteActions[1];
        int secondLeftWheelAction = actionBuffers.DiscreteActions[2];
        int secondRightWheelAction = actionBuffers.DiscreteActions[3];
        int thirdLeftWheelAction = actionBuffers.DiscreteActions[4];
        int thirdRightWheelAction = actionBuffers.DiscreteActions[5];

        // Debug.Log("FL" + firstLeftWheelAction);
        // Debug.Log("FR" + firstRightWheelAction);
        // Debug.Log("SL" + secondLeftWheelAction);
        // Debug.Log("SR" + secondRightWheelAction);
        // Debug.Log("TL" + thirdLeftWheelAction);
        // Debug.Log("TR" + thirdRightWheelAction);

        // z軸が前進(1)後退(-1)
        // firstLeftWheel
        if (firstLeftWheelAction == 0)
        {
            firstLeftWheelControlSignal.z = 1.0f; //前進
        }
        else if (firstLeftWheelAction == 1)
        {
            firstLeftWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].firstLeftWheel.brakeTorque = 100f; //停止
        }

        // firstRightWheel
        if (firstRightWheelAction == 0)
        {
            firstRightWheelControlSignal.z = 1.0f; //前進
        }
        else if (firstRightWheelAction == 1)
        {
            firstRightWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].firstRightWheel.brakeTorque = 100f; //停止
        }

        // secondLeftWheel
        if (secondLeftWheelAction == 0)
        {
            secondLeftWheelControlSignal.z = 1.0f; //前進
        }
        else if (secondLeftWheelAction == 1)
        {
            secondLeftWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].secondLeftWheel.brakeTorque = 100f; //停止
        }

        // secondRightWheel
        if (secondRightWheelAction == 0)
        {
            secondRightWheelControlSignal.z = 1.0f; //前進
        }
        else if (secondRightWheelAction == 1)
        {
            secondRightWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].secondRightWheel.brakeTorque = 100f; //停止
        }

        // thirdLeftWheel
        if (thirdLeftWheelAction == 0)
        {
            thirdLeftWheelControlSignal.z = 1.0f; //前進
        }
        else if (thirdLeftWheelAction == 1)
        {
            thirdLeftWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].thirdLeftWheel.brakeTorque = 100f; //停止
        }

        // thirdRightWheel
        if (thirdRightWheelAction == 0)
        {
            thirdRightWheelControlSignal.z = 1.0f; //前進
        }
        else if (thirdRightWheelAction == 1)
        {
            thirdRightWheelControlSignal.z = -1.0f; //後退
        }
        else
        {
            TwoDMappingaxleInfos[0].thirdRightWheel.brakeTorque = 100f; //停止
        }

        // トルクをかける方向を決定
        float[] motor = new float[6];
        motor[0] = maxMotorTorque * firstLeftWheelControlSignal.z;
        motor[1] = maxMotorTorque * firstRightWheelControlSignal.z;
        motor[2] = maxMotorTorque * secondLeftWheelControlSignal.z;
        motor[3] = maxMotorTorque * secondRightWheelControlSignal.z;
        motor[4] = maxMotorTorque * thirdLeftWheelControlSignal.z;
        motor[5] = maxMotorTorque * thirdRightWheelControlSignal.z;

        // モーター操作
        TwoDMappingaxleInfos[0].firstLeftWheel.motorTorque = motor[0];
        TwoDMappingaxleInfos[0].firstRightWheel.motorTorque = motor[1];
        TwoDMappingaxleInfos[0].secondLeftWheel.motorTorque = motor[2];
        TwoDMappingaxleInfos[0].secondRightWheel.motorTorque = motor[3];
        TwoDMappingaxleInfos[0].thirdLeftWheel.motorTorque = motor[4];
        TwoDMappingaxleInfos[0].thirdRightWheel.motorTorque = motor[5];

        // 9/26追加 Quda
        // LiDARで新規マーキングされたセル数に応じた報酬
        //int newCells = 0; // スコープを広げるために先に宣言
        /*if (lidar != null && lidar.markerManager != null)
        {
            newCells = lidar.GetAndResetNewMarksThisStep();
            if (newCells > 0)
            {
                float cellArea = lidar.markerManager.cellSize * lidar.markerManager.cellSize;
                float reward = newCells * cellArea * areaRewardScale;
                AddReward(reward);
                episodeReward += reward;
            }
        }*/
        // 9/26追加

        // 11/5変更
        // 新規セル数に応じた面積報酬
        int newCellsThisStep = 0;
        // 床ペイントはカメラベースで
        if (useCameraForPainting && camScanner != null)
        {
            camScanner.StepScan(); // このステップで1回だけスキャン
            newCellsThisStep = camScanner.GetAndResetNewMarksThisStep();

            // ターゲット検出（必要なら報酬などをここで）
            if (camScanner.GetAndResetTargetHitThisStep())
            {
                if (targetManager != null && targetManager.isFound)
                {
                    // ここで報酬を入れるなら有効化
                    // AddReward(targetFoundReward);
                    // Debug.Log("Target found (camera)");
                }
            }
        }

        // 面積報酬（新規セルがあった場合のみ）
        if (newCellsThisStep > 0 && meshPainter != null)
        {
            float cellArea = meshPainter.cellSize * meshPainter.cellSize;
            float reward = newCellsThisStep * cellArea * areaRewardScale;
            AddReward(reward);
            episodeReward += reward;
        }
        else
        {
            // 新規セルが増えなかったステップのカウント（比較検証用）
            noNewCellCountThisEpisode++;
        }
        // 11/5変更

        // 停滞ウィンドウの更新 10/9追加
        recentNewCells.Enqueue(newCellsThisStep);
        recentNewCellsSum += newCellsThisStep;
        if (recentNewCells.Count > stagnationWindowSteps)
        {
            recentNewCellsSum -= recentNewCells.Dequeue();
        }

        // 停滞判定とペナルティ
        if (recentNewCells.Count == stagnationWindowSteps && recentNewCellsSum < stagnationMinNewCells)
        {
            AddReward(stagnationPenalty);
            episodeReward += stagnationPenalty;
            stagnationPenaltyCountThisEpisode++; // 回数カウントログ用


            if (endEpisodeOnStagnation)
            {
                // 任意：終了とフラグ更新
                timeUpFlg = 0; // あなたのログフラグ運用に合わせて調整
                EndThisEpisode();
                return;
            }

            // 連続で何度も同じペナルティを受けないよう、ウィンドウをリセットしても良い
            recentNewCells.Clear();
            recentNewCellsSum = 0;
        }

        // 観察窓の更新
        obsRecentNewCells.Enqueue(newCellsThisStep);
        obsRecentNewCellsSum += newCellsThisStep;
        if (obsRecentNewCells.Count > obsWindowSteps)
        {
            obsRecentNewCellsSum -= obsRecentNewCells.Dequeue();
        }
        // 10/9追加

        // 探索率の計算
        float coverageRate = 0f;
        if (lidar != null && lidar.meshPainter != null && totalFloorArea > 0f)
        {
            int markedCells = lidar.meshPainter.GetMarkedCellCount();
            float cellArea = lidar.meshPainter.cellSize * lidar.meshPainter.cellSize;
            float markedArea = markedCells * cellArea;
            //coverageRate = markedArea / totalFloorArea;
            float obstacleArea = (obstacleManager != null) ? obstacleManager.GetTotalObstacleFootprintArea() : 0f; //　11/21追加
            float effectiveFloorArea = Mathf.Max(1e-6f, totalFloorArea - obstacleArea); //　11/21追加 分母補正
            coverageRate = markedArea / effectiveFloorArea; //　11/21追加障害物あり
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

        // 9/29 追加: ログ集計の更新（セル数合計・走行距離）
        newCellsSumThisEpisode += newCellsThisStep;
        // 走行距離の累積（カメラのXZ移動。）
        Vector3 currPos = agentCamera.transform.position;
        distanceTravelled += Vector3.Distance(prevPos, currPos);
        prevPos = currPos;
        // 9/29 追加

        // SnakeRobotAgentが道路から落下した時
        if (Body1.transform.position.y < 0 && Body2.transform.position.y < 0 && Body3.transform.position.y < 0)
        {
            AddReward(-0.5f); // 報酬を-0.5点加算
            fallFlg = 1;
            timeUpFlg = 0;
            fallCount += 1; // フィールドから落下した回数をカウント

            //追加1
            EndThisEpisode();

        }

        // // 車体の角度に応じてマイナスの報酬を与える
        // AddReward(-1e-6f * Mathf.Abs(AngleTranslate(Body1.transform.eulerAngles.y)));
        // AddReward(-1e-6f * Mathf.Abs(AngleTranslate(Body2.transform.eulerAngles.y)));
        // AddReward(-1e-6f * Mathf.Abs(AngleTranslate(Body3.transform.eulerAngles.y)));

    }

    // SnakeRobotAgentが瓦礫に衝突したとき
    public void CollisionChecker(Collision collision)
    {

        if (collision.gameObject.name.Contains("Cube"))
        {
            // 7/02追加 ログ用
            collisionCountLog += 1;
            // 7/02追加

            int idx = episodeCount2 - episodeCount1; // 減算後の値が0始まり
            if (idx >= 0 && idx < collisionCount.Length)
            {
                collisionCount[idx] += 1;
            }

            // 7/02追加 ログ用
            episodeReward += -0.05f; // 報酬を加算（Log用）
            // 7/02追加

            AddReward(-0.05f); // 報酬を-0.05点加算

            /* //衝突時のログを出力 6/18追加
            Debug.Log($"[衝突] オブジェクト名: {collision.gameObject.name}, 衝突回数: {collisionCount[episodeCount2 - (episodeCount1 + 1)]}, 現在の累積報酬: {GetCumulativeReward()}");
            */
            // 6/18追加
        }

    }

    // ヒューリスティックモードの行動決定時に呼ばれる
    public override void Heuristic(in ActionBuffers actionBuffers)
    {

        // Discreteの場合
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

    // 瓦礫に衝突した回数（平均）を算出
    public void averageCollision(int[] collisionCount, int episodeCount2)
    {
        float avgCollision = 0.0f;

        for (int i = 0; i < episodeCount2; i++)//9月29日変更
        {
            avgCollision += collisionCount[i];
            //Debug.Log("瓦礫に衝突した回数[回]：" + avgCollision);
        }
        // avgCollision = avgCollision / episodeCount2;

        // Debug.Log("瓦礫に衝突した回数[回]（平均）：" + avgCollision);
        Debug.Log("瓦礫に衝突した回数[回]：" + avgCollision);
    }

    //ログ　7/15追加
    // 静的変数としてログファイルのパスを共有
    private static string logFilePath = null;

    // ログバッファとロックオブジェクト
    private static List<string> logBuffer = new List<string>();
    private static readonly object logFileLock = new object();
    private const int BufferLimit = 5; // バッファのサイズ
    private static bool isLogFilePathInitialized = false; // ログファイル名が初期化されているかを確認するフラグ
    public string logBaseDir = @"D:\LAB_Drive\Data\KazuyaMaruyama\Logs";


    public void Logger()
    {
        // 初回実行時にのみログファイル名を決定
        if (!isLogFilePathInitialized)
        {
            // ベースフォルダが存在しなければ作成
            if (!Directory.Exists(logBaseDir))
            {
                Directory.CreateDirectory(logBaseDir);
            }

            // 絶対パスで連番ファイル名を取得
            logFilePath = GetNextAvailableFileNameInDir(logBaseDir, "TrainLog", ".csv");
            isLogFilePathInitialized = true;

            // 変更1: 有効なPainterからcellSizeを取得（meshPainter優先、なければlidar.meshPainter）
            var painterRef = (meshPainter != null) ? meshPainter : (lidar != null ? lidar.meshPainter : null);
            float cellSizeForConfig = (painterRef != null) ? painterRef.cellSize : 0f;

            bool newFile = !File.Exists(logFilePath);
            if (newFile)
            {
                // 既存のLiDAR設定のままでも構いません（必要なら後でカメラ用のConfigに拡張）
                string config = $"Config,cellSize,{cellSizeForConfig},numberOfRaysH,{numberOfRaysH},numberOfRaysV,{numberOfRaysV},hFOV,{hFOV},vFOV,{vFOV}";
                File.AppendAllText(logFilePath, config + Environment.NewLine);

                string header = "EpisodeIndex,Steps,MarkedCells,MarkedArea,CoverageRate,StagnationCount,NoNewCellSteps,Reward,Collisions,Fell,Timeout,DistanceTravelled,NewCellsPerStepAvg,Coverage90Step,Coverage90StagCount";
                File.AppendAllText(logFilePath, header + Environment.NewLine);
            }
        }

        try
        {
            if (episodeCountAll > 0)
            {
                // 変更2: 有効なPainter参照に差し替え
                var painterRef = (meshPainter != null) ? meshPainter : (lidar != null ? lidar.meshPainter : null);
                int markedCells = (painterRef != null) ? painterRef.GetMarkedCellCount() : 0;
                float cellSizeLocal = (painterRef != null) ? painterRef.cellSize : 0f;

                float markedArea = markedCells * cellSizeLocal * cellSizeLocal;
                //float coverageRate = (totalFloorArea > 0f) ? (markedArea / totalFloorArea) : 0f;
                float obstacleArea = (obstacleManager != null) ? obstacleManager.GetTotalObstacleFootprintArea() : 0f; //　11/21追加
                float effectiveFloorArea = Mathf.Max(1e-6f, totalFloorArea - obstacleArea); //　11/21追加 分母補正
                float coverageRate = markedArea / effectiveFloorArea; //　11/21追加障害物あり
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

                    // バッファが一定のサイズに達したらファイルに書き込む
                    if (logBuffer.Count >= BufferLimit)
                    {
                        File.AppendAllLines(logFilePath, logBuffer);
                        logBuffer.Clear(); // バッファをクリア
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"ログ書き込みエラー: {ex.Message}");
        }
    }

    // Unity終了時にバッファの内容をすべて書き込む
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

    // 次に利用可能なファイル名を取得
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
using UnityEngine;

public class LiDARScanner2DMapping : MonoBehaviour
{
    // 11/7　追加
    private bool nearestValid = false;           // 最近ヒットが有効か
    private float nearestDist = 0f;              // 最近ヒットの距離（m）
    private float nearestAngleDegLocal = 0f;     // 最近ヒットの角度（度）。ロボット前方0°基準、左負/右正
    // 11/7　追加

    // センサー原点（未設定なら自身）
    public Transform rayOrigin;

    // 水平スキャン設定
    public int numberOfRaysHorizontal = 500;  // 本数：500で1度刻み
    public float horizontalFov = 360f;        // 360なら全周、180なら前方だけ

    // 「世界水平」か「センサー水平」かの選択
    //public bool useWorldHorizontal = true;    // trueで常に地面と平行、falseでセンサー姿勢に追従
    private Vector3 nearestHitPoint = Vector3.zero;

    // 射程・レイヤー
    public float obstacleMaxDistance = 1f;
    public LayerMask obstacleLayer;
    public LayerMask targetLayer;             // 使っていなければ無視

    // 可視化
    public bool drawDebugRays = true;

    public bool drawMagentaEstimated = false; // 紫（推定方向）を描くか

    // 床ペイントはカメラ側に任せるのでここでは無効化
    public bool enableFloorPainting = false;
    public MarkerMeshPainter meshPainter;     // 未使用（enableFloorPainting=trueにした時のみ使用）

    // 11/10追加
    public OccupancyGrid2D occupancyGrid; // 追加：占有グリッド参照
    public Vector2 gridOrigin = new Vector2(0f, 0f);
    public float gridWidth = 5f;
    public float gridDepth = 5f;
    public float gridCellSize = 0.1f;
    // 11/10追加

    // 1ステップ新規ペイント数（床ペイント無効なら常に0）
    private int newMarksThisStep = 0;
    public int GetAndResetNewMarksThisStep()
    {
        int n = newMarksThisStep;
        newMarksThisStep = 0;
        return n;
    }

    private void DrawNearest()
    {
        if (!nearestValid) return;

        Transform tf = (rayOrigin != null) ? rayOrigin : transform;
        Vector3 originPos = tf.position;

        // 角度から作る可視化ベクトル（ロボット前方基準）
        Vector3 dirAngle = Quaternion.AngleAxis(nearestAngleDegLocal, tf.up) * tf.forward;

        // 推定方向（マゼンタ）
        //Debug.DrawRay(originPos, dirAngle.normalized * nearestDist, Color.magenta);

        // 実ヒット点へのライン（グリーン）で整合確認
        Debug.DrawLine(originPos, nearestHitPoint, Color.green);
    }

    // 11/6追加: 最近障害物観察用の内部状態
    private void ScanHorizontal()
    {
        Transform tf = (rayOrigin != null) ? rayOrigin : transform;
        Vector3 originPos = tf.position;

        // 毎スキャン初期化（宣言はしない）
        nearestValid = false;
        nearestDist = obstacleMaxDistance;
        nearestAngleDegLocal = 0f;

        float yawStep = horizontalFov / Mathf.Max(1, numberOfRaysHorizontal);
        for (int i = 0; i < numberOfRaysHorizontal; i++)
        {
            float yawDeg = -horizontalFov / 2f + i * yawStep;

            // ローカルupで回し、ローカルforward基準の水平レイ
            Vector3 dir = Quaternion.AngleAxis(yawDeg, tf.up) * tf.forward;
            Ray ray = new Ray(originPos, dir.normalized);
            bool obsHit = Physics.Raycast(ray, out var hitObs, obstacleMaxDistance, obstacleLayer);

            // 追加: 占有グリッド更新（ヒットまでfree、ヒットセルoccupied）
            if (occupancyGrid != null)
            {
                float maxDist = obstacleMaxDistance;
                float hitDist = obsHit ? hitObs.distance : maxDist;
                occupancyGrid.IntegrateRay(originPos, dir.normalized, maxDist, obsHit, hitDist);
            }

            if (obsHit)
            {
                Vector3 toHit = hitObs.point - originPos;
                Vector2 fwdXZ = new Vector2(tf.forward.x, tf.forward.z).normalized;
                Vector2 toHitXZ = new Vector2(toHit.x, toHit.z).normalized;
                float angleDegLocal = Vector2.SignedAngle(fwdXZ, toHitXZ);

                if (hitObs.distance < nearestDist)
                {
                    nearestDist = hitObs.distance;
                    nearestAngleDegLocal = angleDegLocal;
                    nearestHitPoint = hitObs.point;
                    nearestValid = true;
                }
            }

            if (drawDebugRays)
            {
                if (obsHit) Debug.DrawLine(ray.origin, hitObs.point, Color.blue);
                else Debug.DrawRay(ray.origin, dir.normalized * obstacleMaxDistance, Color.green);
            }
        }


    }

    public (bool has, float distNorm, float angleNorm) GetNearestObstacleObservation()
    {
        if (!nearestValid) return (false, 1f, 0f);
        float distNorm = Mathf.Clamp01(nearestDist / Mathf.Max(0.0001f, obstacleMaxDistance));
        float halfFov = Mathf.Max(1e-3f, horizontalFov * 0.5f);
        float angleNorm = Mathf.Clamp(nearestAngleDegLocal, -halfFov, halfFov) / halfFov;
        return (true, distNorm, angleNorm);
    }


    public bool drawNearestArrow = true; // 11/7　追加 InspectorでON/OFF

    // 11/13追加　初期化用
    public void ClearGrid()
    {
        if (occupancyGrid != null)
            occupancyGrid.Clear();
    }

    void Awake()
    {
        if (occupancyGrid == null) occupancyGrid = new OccupancyGrid2D();
        occupancyGrid.Initialize(gridOrigin, gridWidth, gridDepth, gridCellSize);
    }

    void Update()
    {
        ScanHorizontal(); // 最近距離・角度を更新
        DrawNearest();    // 直後に可視化（同じtfを使う）
    }
}
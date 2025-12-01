using UnityEngine;
using System.Collections.Generic;

public class ObstacleManager : MonoBehaviour
{
    [Header("Prefabs")]
    public GameObject tallObstaclePrefab;   // 背の高い（LiDAR検知される）
    public GameObject lowObstaclePrefab;    // 背の低い（LiDAR検知されない想定）

    [Header("Counts")]
    public int tallCount = 5;
    public int lowCount = 5;

    [Header("Placement (rectangle)")]
    public Vector2 origin = new Vector2(-0.3f, -0.3f); // 左下
    public float width = 5f;
    public float depth = 5f;
    public float minSpacing = 1f;
    public float boundsMargin = 0.2f;

    [Header("Placement constraints")]
    public float minDistanceFromOrigin = 2.0f;  // 原点からの最小距離[m]
    public Vector2 forbiddenCenter = Vector2.zero; // 原点(0,0)をXZで指定

    [Header("LiDAR height check (debug)")]
    public float lidarHeightY = 0.2f; // LiDARの世界高さ。あなたのLiDARセンサーのTransform.position.yに合わせる

    [Header("Collider sizing (optional override)")]
    public float tallHeight = 0.5f; // Tallの物体高さ（コライダ用目安）
    public float lowHeight = 0.05f; // Lowの物体高さ（LiDAR高さより低くする）
    public bool overrideColliderSize = false; // trueなら生成時にCollider高さを上書き調整

    [Header("Collision check layers")]
    public LayerMask blockingLayers; // 重なり防止に使う（Walls/Robotなど）

    private readonly List<GameObject> spawnedTall = new List<GameObject>();
    private readonly List<GameObject> spawnedLow = new List<GameObject>();

    // 12/1追加　評価用　障害物固定
    public bool useFixedPlacement = false;
    // 固定配置座標（Tall/Lowで別管理する例）
    public Vector3[] fixedTallPositions;
    public Vector3[] fixedLowPositions;
    // 固定配置の回転（任意）
    public float[] fixedTallYaws;
    public float[] fixedLowYaws;
    // 12/1追加　評価用　障害物固定

    // 既に配置した中心座標を記録（Tall/Low共通）
    private readonly List<Vector3> placedCenters = new List<Vector3>();

    // 記録済み中心との最短距離が閾値以上か（近過ぎないか）
    private bool IsFarFromPlacedCenters(Vector3 candidateCenter, float minDistance)
    {
        for (int i = 0; i < placedCenters.Count; i++)
        {
            if (Vector3.Distance(candidateCenter, placedCenters[i]) < minDistance)
                return false; // 近すぎる
        }
        return true;
    }

    // 12/1追加　評価用　障害物固定
    private void PlaceFixed()
    {
        // Tall
        int nTall = Mathf.Min(spawnedTall.Count, fixedTallPositions != null ? fixedTallPositions.Length : 0);
        for (int i = 0; i < nTall; i++)
        {
            var go = spawnedTall[i];
            Vector3 p = fixedTallPositions[i];
            float yaw = (fixedTallYaws != null && i < fixedTallYaws.Length) ? fixedTallYaws[i] : 0f;
            go.transform.SetPositionAndRotation(new Vector3(p.x, 0f, p.z), Quaternion.Euler(0f, yaw, 0f));
        }

        // Low
        int nLow = Mathf.Min(spawnedLow.Count, fixedLowPositions != null ? fixedLowPositions.Length : 0);
        for (int i = 0; i < nLow; i++)
        {
            var go = spawnedLow[i];
            Vector3 p = fixedLowPositions[i];
            float yaw = (fixedLowYaws != null && i < fixedLowYaws.Length) ? fixedLowYaws[i] : 0f;
            go.transform.SetPositionAndRotation(new Vector3(p.x, 0f, p.z), Quaternion.Euler(0f, yaw, 0f));
        }
    }

    public void ResetRandom()
    {
        if (useFixedPlacement)
        {
            PlaceFixed(); // 固定配置関数
        }
        else
        {
            // Tall + Low をまとめて再配置
            var all = new List<GameObject>(spawnedTall.Count + spawnedLow.Count);
            all.AddRange(spawnedTall);
            all.AddRange(spawnedLow);
            RelocateRandomAll(all); // 統合ランダム配置
        }
    }
    // 12/1追加　評価用　障害物固定

    public float GetTotalObstacleFootprintArea()
    {
        float sum = 0f;
        foreach (var go in spawnedTall) sum += ComputeFootprintArea(go);
        foreach (var go in spawnedLow) sum += ComputeFootprintArea(go);
        return sum;
    }

    private float ComputeFootprintArea(GameObject go)
    {
        // 代表のColliderを取る（複数ある場合は最も広いXZ面積を採用する例）
        var colliders = go.GetComponents<Collider>();
        if (colliders == null || colliders.Length == 0) return 0f;

        float maxArea = 0f;
        foreach (var col in colliders)
        {
            float area = 0f;
            var t = col.transform;
            Vector3 lossy = t.lossyScale;

            if (col is BoxCollider box)
            {
                Vector3 sizeW = Vector3.Scale(box.size, lossy);
                area = Mathf.Abs(sizeW.x * sizeW.z);
            }
            else if (col is CapsuleCollider cap)
            {
                // Capsuleの投影は半径の円。axis=Y想定（縦カプセル）
                float radius = cap.radius * Mathf.Max(lossy.x, lossy.z);
                area = Mathf.PI * radius * radius;
            }
            else if (col is SphereCollider sph)
            {
                float radius = sph.radius * Mathf.Max(lossy.x, lossy.z);
                area = Mathf.PI * radius * radius;
            }
            else if (col is MeshCollider meshCol)
            {
                // MeshColliderは形状が複雑。AABB近似を使う
                var bounds = meshCol.sharedMesh != null ? meshCol.sharedMesh.bounds : new Bounds(Vector3.zero, Vector3.zero);
                Vector3 sizeLocal = bounds.size;
                Vector3 sizeW = Vector3.Scale(sizeLocal, lossy);
                area = Mathf.Abs(sizeW.x * sizeW.z);
                // 必要なら補正係数（convexでない場合は過大/過小になることがある）
            }

            if (area > maxArea) maxArea = area;
        }
        return maxArea;
    }

    void Start()
    {
        SpawnAll();
    }

    // すべての障害物（Tall+Low）をまとめて再配置
    //public void ResetRandom()
    //{
    //    var all = new List<GameObject>(spawnedTall.Count + spawnedLow.Count);
    //    all.AddRange(spawnedTall);
    //    all.AddRange(spawnedLow);
    //    RelocateRandomAll(all); //統合版を呼ぶ
    //}

    private void SpawnAll()
    {
        for (int i = 0; i < tallCount; i++)
        {
            var go = Instantiate(tallObstaclePrefab);
            // 必要ならレイヤ設定
            // go.layer = LayerMask.NameToLayer("Obstacles");
            spawnedTall.Add(go);
            if (overrideColliderSize) SetupColliderHeight(go, tallHeight);
        }

        for (int i = 0; i < lowCount; i++)
        {
            var go = Instantiate(lowObstaclePrefab);
            // go.layer = LayerMask.NameToLayer("Obstacles");
            spawnedLow.Add(go);
            if (overrideColliderSize) SetupColliderHeight(go, lowHeight);
        }

        // 統合配置を呼ぶ
        var all = new List<GameObject>(spawnedTall.Count + spawnedLow.Count);
        all.AddRange(spawnedTall);
        all.AddRange(spawnedLow);
        RelocateRandomAll(all); // ← ここに統合版を呼ぶ
    }

    // 統合版の配置関数（前回答のRelocateRandomAllをクラス内に定義しておく）
    private void RelocateRandomAll(List<GameObject> objects)
    {
        int triesMax = 300;

        // 既存記録はクリア（毎回ランダム再配置するなら）
        placedCenters.Clear();

        for (int idx = 0; idx < objects.Count; idx++)
        {
            var go = objects[idx];
            bool placed = false;

            var box = go.GetComponent<BoxCollider>();
            // オブジェクトごとの「距離閾値」を動的に設定（半サイズ＋minSpacing）
            float minDist = minSpacing;
            Vector3 halfExtentsForDist = Vector3.zero;
            Vector3 centerOffset = Vector3.zero;
            if (box != null)
            {
                halfExtentsForDist = Vector3.Scale(box.size, go.transform.lossyScale) * 0.5f;
                // X/Zの半サイズのうち大きい方＋minSpacingを距離閾値に採用
                minDist = Mathf.Max(halfExtentsForDist.x, halfExtentsForDist.z) * 2f + minSpacing * 0.5f;
                centerOffset = box.center;
            }

            for (int attempt = 0; attempt < triesMax; attempt++)
            {
                Vector3 pos = SamplePositionInsideBounds();
                if (!IsInsideWithMargin(pos)) continue;

                // 原点距離制約
                if (!IsOutsideForbiddenRadius(pos, minDistanceFromOrigin, forbiddenCenter)) continue;

                Vector3 candidateCenter = pos + centerOffset;

                // 1) 記録座標との距離チェック（軽量）
                if (!IsFarFromPlacedCenters(candidateCenter, minDist))
                    continue;

                // 2) 形状準拠の重なり検査（より厳密）
                bool overlap;
                if (box != null)
                {
                    Vector3 halfExtents = halfExtentsForDist + new Vector3(minSpacing * 0.5f, 0f, minSpacing * 0.5f);
                    overlap = Physics.CheckBox(candidateCenter,
                                               halfExtents,
                                               Quaternion.identity,
                                               blockingLayers,
                                               QueryTriggerInteraction.Ignore);
                }
                else
                {
                    overlap = Physics.CheckSphere(pos, minSpacing, blockingLayers, QueryTriggerInteraction.Ignore);
                }
                if (overlap) continue;

                // OKなら配置
                go.transform.SetPositionAndRotation(pos, Quaternion.Euler(0f, Random.Range(0f, 0f), 0f));
                placedCenters.Add(candidateCenter); // 記録
                placed = true;
                break;
            }

            if (!placed)
                Debug.LogWarning($"ObstacleManager: failed to place {go.name} after {triesMax} tries.");
        }
    }

    // 原点から半径minDist外にあるか（XZ平面）
    private bool IsOutsideForbiddenRadius(Vector3 p, float minDist, Vector2 centerXZ)
    {
        float dx = p.x - centerXZ.x;
        float dz = p.z - centerXZ.y;
        return (dx * dx + dz * dz) >= (minDist * minDist);
    }

    private Vector3 SamplePositionInsideBounds()
    {
        float rx = origin.x + Random.value * width;
        float rz = origin.y + Random.value * depth;
        return new Vector3(rx, 0f, rz);
    }

    private bool IsInsideWithMargin(Vector3 p)
    {
        return (p.x >= origin.x + boundsMargin &&
                p.x <= origin.x + width - boundsMargin &&
                p.z >= origin.y + boundsMargin &&
                p.z <= origin.y + depth - boundsMargin);
    }

    // 生成時にCollider高さを上書きしたい場合
    private void SetupColliderHeight(GameObject go, float targetHeight)
    {
        // BoxCollider想定（必要ならCapsule/Sphereにも対応）
        var box = go.GetComponent<BoxCollider>();
        if (box != null)
        {
            Vector3 size = box.size;
            size.y = targetHeight;
            box.size = size;

            // 中心を床からの高さの半分に調整（底面が床に乗る想定）
            Vector3 center = box.center;
            center.y = targetHeight * 0.5f;
            box.center = center;
            return;
        }

        var capsule = go.GetComponent<CapsuleCollider>();
        if (capsule != null)
        {
            capsule.height = targetHeight;
            Vector3 center = capsule.center;
            center.y = targetHeight * 0.5f;
            capsule.center = center;
            return;
        }

        var sphere = go.GetComponent<SphereCollider>();
        if (sphere != null)
        {
            sphere.radius = targetHeight * 0.5f;
            Vector3 center = sphere.center;
            center.y = targetHeight * 0.5f;
            sphere.center = center;
            return;
        }

        // MeshColliderはconvexでないとRaycastに扱いづらい。できればBox/Capsuleに置き換え推奨
    }

    // デバッグ用：LiDAR高さで当たるかどうかの判定（色を変えるなど）
    private void ValidateAgainstLidarHeight(List<GameObject> list, bool shouldBeDetected)
    {
        foreach (var go in list)
        {
            var box = go.GetComponent<BoxCollider>();
            float topY = 0f;
            float bottomY = 0f;

            if (box != null)
            {
                // ワールド座標で上下端を計算
                var t = box.transform;
                float halfH = box.size.y * 0.5f;
                float centerYWorld = t.position.y + box.center.y;
                bottomY = centerYWorld - halfH;
                topY = centerYWorld + halfH;
            }
            else
            {
                // 他Colliderでも同様に計算（省略）
            }

            bool hitsLidarPlane = (bottomY <= lidarHeightY && lidarHeightY <= topY);
            if (shouldBeDetected && !hitsLidarPlane)
                Debug.LogWarning($"{go.name} should be detected but misses LiDAR plane (topY={topY:F3}, bottomY={bottomY:F3}, lidarY={lidarHeightY:F3})");
            if (!shouldBeDetected && hitsLidarPlane)
                Debug.LogWarning($"{go.name} should NOT be detected but intersects LiDAR plane");
        }
    }
}
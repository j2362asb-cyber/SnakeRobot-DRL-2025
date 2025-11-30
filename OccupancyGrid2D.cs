using UnityEngine;
using System;

[System.Serializable] // これを追加
public class OccupancyGrid2D
{
    public Vector2 origin = new Vector2(0f, 0f); // グリッド原点（X,Z）
    public float width = 5f;                      // X方向[m]
    public float depth = 5f;                      // Z方向[m]
    public float cellSize = 0.1f;                 // セル[m]

    private int xCells, zCells;
    private float[,] logOdds;                     // ロジット（log(p/(1-p))）
    private bool[,] everUpdated;                  // 未知判定用（更新されたことがあるか）

    // センサーモデル（調整用）
    public float pHit = 0.7f;   // ヒット時に占有へ寄せる強さ
    public float pMiss = 0.4f;  // ヒットまでのセルを自由へ寄せる強さ
    public float clampMin = -4f, clampMax = 4f; // ロジットのクランプ

    // 11/11追加
    public float OccupancyThreshold = 0.7f; // 確定占有と見なす閾値（調整可）
    public bool IsOccupied(int cx, int cz)
    {
        return GetProb(cx, cz) >= OccupancyThreshold;
    }
    // 11/11追加

    private float lHit, lMiss;

    public void Initialize(Vector2 origin, float width, float depth, float cellSize)
    {
        this.origin = origin; this.width = width; this.depth = depth; this.cellSize = cellSize;
        xCells = Mathf.CeilToInt(width / cellSize);
        zCells = Mathf.CeilToInt(depth / cellSize);
        logOdds = new float[xCells, zCells];
        everUpdated = new bool[xCells, zCells];

        // 初期は未知 → ロジット0（p=0.5）
        for (int x = 0; x < xCells; x++)
            for (int z = 0; z < zCells; z++)
                logOdds[x, z] = 0f;

        // センサーモデルのロジット差分
        lHit = Mathf.Log(pHit / (1f - pHit));
        lMiss = Mathf.Log(pMiss / (1f - pMiss));
    }

    // ワールド座標→セル
    public bool WorldPosToCell(Vector3 worldPos, out int cx, out int cz)
    {
        float localX = worldPos.x - origin.x;
        float localZ = worldPos.z - origin.y;
        cx = Mathf.FloorToInt(localX / cellSize);
        cz = Mathf.FloorToInt(localZ / cellSize);
        if (cx < 0 || cx >= xCells || cz < 0 || cz >= zCells) return false;
        return true;
    }

    // レイ統合：originからdirに距離maxDistまで。hitDistがあればそこを占有、なければ最後まで自由。
    public int IntegrateRay(Vector3 originWorld, Vector3 dirWorld, float maxDist, bool hasHit, float hitDist)
    {
        int updatedCells = 0;
        float step = cellSize * 0.5f;
        float travel = 0f;

        while (travel < maxDist)
        {
            Vector3 p = originWorld + dirWorld * travel;
            if (!WorldPosToCell(p, out int cx, out int cz)) { travel += step; continue; }

            // ヒット前はfreeへ寄せたいが、既に占有確定ならfree更新をスキップ
            if (!IsOccupied(cx, cz))
            {
                AddLogOdds(cx, cz, lMiss);
                updatedCells++;
            }

            travel += step;

            if (hasHit && travel >= hitDist)
            {
                Vector3 hitP = originWorld + dirWorld * hitDist;
                if (WorldPosToCell(hitP, out int hcX, out int hcZ))
                {
                    AddLogOdds(hcX, hcZ, lHit); // ヒットは常に占有へ
                    updatedCells++;
                }
                break;
            }
        }
        return updatedCells;
    }

    private void AddLogOdds(int cx, int cz, float delta)
    {
        float l = logOdds[cx, cz] + delta;
        l = Mathf.Clamp(l, clampMin, clampMax);
        logOdds[cx, cz] = l;
        everUpdated[cx, cz] = true;
    }

    // 確率へ変換
    public float GetProb(int cx, int cz)
    {
        // p = 1 / (1 + exp(-l))
        float l = logOdds[cx, cz];
        return 1f / (1f + Mathf.Exp(-l));
    }

    public bool IsInsideCell(int cx, int cz) => (cx >= 0 && cx < xCells && cz >= 0 && cz < zCells);

    // 観察用：ロボット中心・進行方向基準の正方パッチ（N×N, 偶数でもOK） 11/12変更
    public float[] GetEgocentricPatch(Vector3 agentPos, Vector3 agentForward, int patchCells, out int widthOut, out int heightOut)
    {
        widthOut = patchCells; heightOut = patchCells;
        float[] data = new float[patchCells * patchCells];

        // 進行方向（XZ）と右方向
        Vector2 fwd = new Vector2(agentForward.x, agentForward.z);
        if (fwd.sqrMagnitude < 1e-8f) fwd = Vector2.up;
        fwd.Normalize();
        Vector2 right = new Vector2(fwd.y, -fwd.x); // 90度回転（右手座標）

        int half = patchCells / 2;
        for (int ix = -half; ix < half; ix++)
        {
            for (int iz = -half; iz < half; iz++)
            {
                Vector2 offsetXZ = right * (ix * cellSize) + fwd * (iz * cellSize);
                Vector3 world = new Vector3(agentPos.x + offsetXZ.x, agentPos.y, agentPos.z + offsetXZ.y);

                float val = 0.5f; // 未知デフォルト
                if (WorldPosToCell(world, out int cx, out int cz))
                {
                    val = GetProb(cx, cz); // 0..1（低→free, 高→occupied）
                }

                int xIndex = ix + half;      // 左→右
                int zIndex = iz + half;      // 手前→前（進行方向上）
                data[zIndex * patchCells + xIndex] = Mathf.Clamp01(val);
            }
        }
        return data;
    }

    // 探索率（更新済みセルの割合）
    public float GetCoverageRate()
    {
        int updated = 0;
        int total = xCells * zCells;
        for (int x = 0; x < xCells; x++)
            for (int z = 0; z < zCells; z++)
                if (everUpdated[x, z]) updated++;
        return (total > 0) ? (float)updated / total : 0f;
    }

    // 11/13追加　初期化処理
    public void Clear()
    {
        for (int x = 0; x < XCells; x++)
            for (int z = 0; z < ZCells; z++)
            {
                logOdds[x, z] = 0f;      // p=0.5（未知）
                everUpdated[x, z] = false;
            }
    }

    public int XCells => xCells;
    public int ZCells => zCells;
}

using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Rendering;

public class MarkerMeshPainter : MonoBehaviour
{
    [Header("Grid (world space)")]
    public Vector2 origin = new Vector2(0f, 0f);      // グリッド原点（X,Z）
    public float width = 5f;                          // X方向の長さ（m）
    public float depth = 5f;                          // Z方向の長さ（m）
    public float cellSize = 0.1f;                     // セルサイズ（m）

    [Header("Visual")]
    public float yOffset = 0.001f;                    // 床からわずかに浮かせる
    public Material markerMaterial;                   // 頂点カラー表示用マテリアル
    public Color paintedColor = new Color(1f, 1f, 0f, 1f); // 黄色
    public Color unpaintedColor = new Color(0f, 0f, 0f, 0f); // 透過（黒）

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;

    private Mesh mesh;
    private Color[] colors;

    // セル数
    private int xCells;
    private int zCells;

    // セル→頂点インデックス4個へのマップ
    private int[,] cellToVertexBaseIndex; // 各セルの左下頂点のインデックス（Quadの4頂点は +0,+1,+2,+3）

    // 既踏判定（O(1)）
    private bool[,] visited;

    void Awake()
    {
        meshFilter = gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null) meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.GetComponent<MeshRenderer>();
        if (meshRenderer == null) meshRenderer = gameObject.AddComponent<MeshRenderer>();

        meshRenderer.sharedMaterial = markerMaterial;

        BuildGridMesh();
        ClearAll(); // 初期は未塗り
    }

    // グリッドメッシュ生成（各セル1 Quad = 4頂点、6インデックス）
    void BuildGridMesh()
    {
        xCells = Mathf.CeilToInt(width / cellSize);
        zCells = Mathf.CeilToInt(depth / cellSize);
        // 端に合わせたい場合。境界を確実に含めたいなら CeilToInt のままでもOK

        int vertexCount = xCells * zCells * 4;
        int indexCount = xCells * zCells * 6;

        Vector3[] vertices = new Vector3[vertexCount];
        Vector2[] uvs = new Vector2[vertexCount]; // 任意。今回は未使用でも可
        int[] indices = new int[indexCount];
        colors = new Color[vertexCount];

        cellToVertexBaseIndex = new int[xCells, zCells];
        visited = new bool[xCells, zCells];

        int v = 0;
        int t = 0;

        for (int x = 0; x < xCells; x++)
        {
            for (int z = 0; z < zCells; z++)
            {
                // セルの左下ワールド座標（XZ）
                float x0 = origin.x + x * cellSize;
                float z0 = origin.y + z * cellSize;

                // 4頂点（QuadをXZ平面に配置）
                // 左下
                vertices[v + 0] = new Vector3(x0, yOffset, z0);
                // 右下
                vertices[v + 1] = new Vector3(x0 + cellSize, yOffset, z0);
                // 左上
                vertices[v + 2] = new Vector3(x0, yOffset, z0 + cellSize);
                // 右上
                vertices[v + 3] = new Vector3(x0 + cellSize, yOffset, z0 + cellSize);

                // UVは任意（0..1で割り当て）
                uvs[v + 0] = new Vector2((float)x / xCells, (float)z / zCells);
                uvs[v + 1] = new Vector2((float)(x + 1) / xCells, (float)z / zCells);
                uvs[v + 2] = new Vector2((float)x / xCells, (float)(z + 1) / zCells);
                uvs[v + 3] = new Vector2((float)(x + 1) / xCells, (float)(z + 1) / zCells);

                // Quadのインデックス（2トライアングル）
                // 三角形1: 0,2,1
                indices[t + 0] = v + 0;
                indices[t + 1] = v + 2;
                indices[t + 2] = v + 1;
                // 三角形2: 2,3,1
                indices[t + 3] = v + 2;
                indices[t + 4] = v + 3;
                indices[t + 5] = v + 1;

                // 頂点カラー初期値（未塗り）
                colors[v + 0] = unpaintedColor;
                colors[v + 1] = unpaintedColor;
                colors[v + 2] = unpaintedColor;
                colors[v + 3] = unpaintedColor;

                // セル→頂点開始インデックスを記録
                cellToVertexBaseIndex[x, z] = v;

                v += 4;
                t += 6;
            }
        }

        mesh = new Mesh();
        mesh.indexFormat = IndexFormat.UInt32; // 大規模頂点対応
        mesh.vertices = vertices;
        mesh.uv = uvs;
        mesh.triangles = indices;
        mesh.colors = colors;
        mesh.RecalculateBounds();
        mesh.RecalculateNormals(); // Unlitでも陰影は不要ですが、保険で

        meshFilter.sharedMesh = mesh;
    }

    // 座標→セルインデックス変換（ワールドXZ）// 10/3変更
    private bool WorldPosToCell(Vector3 worldPos, out int cx, out int cz)
    {
        float localX = worldPos.x - origin.x;
        float localZ = worldPos.z - origin.y;

        // クランプはやめて、直接インデックス計算
        cx = Mathf.FloorToInt(localX / cellSize);
        cz = Mathf.FloorToInt(localZ / cellSize);

        // 右端・上端に届くヒット点は、localX==widthやlocalZ==depth付近で
        // ちょうど ix == xCells や iz == zCells になり得るので、Clampで救済
        cx = Mathf.Clamp(cx, 0, xCells - 1);
        cz = Mathf.Clamp(cz, 0, zCells - 1);

        return true;
    }

    // 1セルを着色（新規ならtrue）
    public bool PaintAtWorldPosition(Vector3 worldPos)
    {
        // まず端誤差込みでセルに落とす
        if (!WorldPosToCell(worldPos, out int cx, out int cz))
            return false;

        if (visited[cx, cz]) return false;
        visited[cx, cz] = true;

        // セル中心のワールド座標にスナップ（右端・上端も確実に塗られる）
        float xCenter = origin.x + (cx + 0.5f) * cellSize;
        float zCenter = origin.y + (cz + 0.5f) * cellSize;
        Vector3 snapped = new Vector3(xCenter, yOffset, zCenter);

        // 頂点カラー更新
        int baseIndex = cellToVertexBaseIndex[cx, cz];
        colors[baseIndex + 0] = paintedColor;
        colors[baseIndex + 1] = paintedColor;
        colors[baseIndex + 2] = paintedColor;
        colors[baseIndex + 3] = paintedColor;
        mesh.colors = colors;

        return true;
    }

    // クリア（未塗りに戻す）
    public void ClearAll()
    {
        for (int x = 0; x < xCells; x++)
        {
            for (int z = 0; z < zCells; z++)
                visited[x, z] = false;
        }
        for (int i = 0; i < colors.Length; i++)
            colors[i] = unpaintedColor;
        mesh.colors = colors;
    }

    // 進捗取得（ログ用など）
    public int GetMarkedCellCount()
    {
        int count = 0;
        for (int x = 0; x < xCells; x++)
            for (int z = 0; z < zCells; z++)
                if (visited[x, z]) count++;
        return count;
    }

    public float GetMarkedArea()
    {
        return GetMarkedCellCount() * (cellSize * cellSize);
    }
    public Vector2 GetUnvisitedCentroidXZ()
    {
        int count = 0;
        double sumX = 0.0, sumZ = 0.0;

        // グリッド内を走査して未訪問セルの中心を合計
        for (int x = 0; x < xCells; x++)
        {
            for (int z = 0; z < zCells; z++)
            {
                if (!visited[x, z])
                {
                    float cx = origin.x + (x + 0.5f) * cellSize;
                    float cz = origin.y + (z + 0.5f) * cellSize;
                    sumX += cx;
                    sumZ += cz;
                    count++;
                }
            }
        }

        if (count == 0) return Vector2.zero;
        return new Vector2((float)(sumX / count), (float)(sumZ / count));
    }
}
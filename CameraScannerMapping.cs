using UnityEngine;

public class CameraScannerMapping : MonoBehaviour
{
    [Header("Camera")]
    public Camera sourceCamera;

    [Header("Sampling (screen grid)")]
    public int samplesX = 64;         // 横方向サンプル数（負荷と精度のトレードオフ）
    public int samplesY = 36;         // 縦方向サンプル数
    public bool drawDebugRays = false;

    [Header("Ranges")]
    public float floorMaxDistance = 1f;
    public float targetMaxDistance = 1f;

    [Header("Layers")]
    public LayerMask floorLayer;
    public LayerMask targetLayer;

    [Header("Painting / Target")]
    public MarkerMeshPainter meshPainter;
    public TargetManager targetManager;

    private int newMarksThisStep = 0;
    private bool targetHitThisStep = false; // このステップでターゲットが見つかったか

    public int GetAndResetNewMarksThisStep()
    {
        int n = newMarksThisStep;
        newMarksThisStep = 0;
        return n;
    }

    public bool GetAndResetTargetHitThisStep()
    {
        bool v = targetHitThisStep;
        targetHitThisStep = false;
        return v;
    }

    public bool manualStep = true;

    void Update()
    {
        // manualStep が false のときだけ毎フレームスキャン
        if (!manualStep)
        {
            ScanCameraVisibility();
            PaintFloorDirectlyDown();
        }
    }

    // Agent から1ステップにつき1回呼ぶ用
    public void StepScan()
    {
        ScanCameraVisibility();
        PaintFloorDirectlyDown();
    }

    void ScanCameraVisibility()
    {
        if (meshPainter == null || sourceCamera == null) return;

        int newMarks = 0;

        // カメラのピクセル矩形（Viewportが0..1以外の場合に備えて）
        Rect pixelRect = sourceCamera.pixelRect;

        // 画面を格子でサンプリング
        for (int i = 0; i < samplesX; i++)
        {
            for (int j = 0; j < samplesY; j++)
            {
                float u = (i + 0.5f) / samplesX;
                float v = (j + 0.5f) / samplesY;

                float px = pixelRect.x + u * pixelRect.width;
                float py = pixelRect.y + v * pixelRect.height;

                Ray ray = sourceCamera.ScreenPointToRay(new Vector3(px, py, 0f));

                //　11/21追加
                if (Physics.Raycast(ray, out var hitAny, floorMaxDistance, ~0)) // 全レイヤ
                {
                    if (((1 << hitAny.collider.gameObject.layer) & floorLayer) != 0)
                    {
                        // 最初のヒットが床 → 可視（遮蔽済み）
                        var p = hitAny.point; p.y = meshPainter.yOffset;
                        if (meshPainter.PaintAtWorldPosition(p)) newMarks++;
                    }
                    else
                    {
                        // 最初のヒットが障害物など → 床は見えていないので塗らない
                    }
                }
                //　11/21追加

                //// ターゲット検出（未発見の場合のみ）
                //if (targetManager != null && !targetManager.isFound)
                //{
                //    if (Physics.Raycast(ray, out var hitT, targetMaxDistance, targetLayer))
                //    {
                //        targetManager.MarkFound();
                //        targetHitThisStep = true;
                //    }
                //}

                //// 床ヒット → ペイント（遮蔽物で床が見えない場合は塗られない）
                //if (Physics.Raycast(ray, out var hitFloor, floorMaxDistance, floorLayer))
                //{
                //    Vector3 p = hitFloor.point;
                //    p = new Vector3(p.x, meshPainter.yOffset, p.z);
                //    if (meshPainter.PaintAtWorldPosition(p))
                //        newMarks++;

                //    if (drawDebugRays)
                //        Debug.DrawLine(ray.origin, hitFloor.point, Color.yellow);
                //}
                //else if (drawDebugRays)
                //{
                //    Debug.DrawRay(ray.origin, ray.direction , Color.cyan);
                //}
            }
        }

        newMarksThisStep = newMarks;
    }

    void PaintFloorDirectlyDown()
    {
        if (meshPainter == null || sourceCamera == null) return;

        Vector3 origin = sourceCamera.transform.position + Vector3.up * 0.05f;
        if (Physics.Raycast(origin, Vector3.down, out var hit, floorMaxDistance, floorLayer))
        {
            Vector3 p = hit.point;
            p = new Vector3(p.x, meshPainter.yOffset, p.z);
            meshPainter.PaintAtWorldPosition(p);
        }
    }
}

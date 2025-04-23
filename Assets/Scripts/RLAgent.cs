using UnityEngine;
using Unity.Barracuda;

public class RLAgent : MonoBehaviour
{
    [Header("Model & Inference")]
    public NNModel investigationModel;
    private IWorker worker;

    [Header("Grid Reference")]
    public GridManager gridManager;

    void Awake()
    {
        // Load the model and create a worker
        var model = ModelLoader.Load(investigationModel);
        worker = WorkerFactory.CreateWorker(WorkerFactory.Type.Auto, model);
    }

    public Vector3 GetNextInvestigationNode(Vector3 guardPos, Vector3 lastSeenPlayerPos)
    {
        Node guardNode = gridManager.NodeFromWorldPoint(guardPos);
        Node targetNode = gridManager.NodeFromWorldPoint(lastSeenPlayerPos);

        float gx = guardNode.gridX    / (float)(gridManager.gridSizeX - 1);
        float gy = guardNode.gridY    / (float)(gridManager.gridSizeY - 1);
        float tx = targetNode.gridX   / (float)(gridManager.gridSizeX - 1);
        float ty = targetNode.gridY   / (float)(gridManager.gridSizeY - 1);

        using (var input = new Tensor(1, 4))
        {
            input[0, 0] = gx;
            input[0, 1] = gy;
            input[0, 2] = tx;
            input[0, 3] = ty;

            worker.Execute(input);

            Tensor output = worker.PeekOutput();  
            int action = ArgMax(output);
            output.Dispose();
            input.Dispose();
            int nx = guardNode.gridX;
            int ny = guardNode.gridY;
            switch (action)
            {
                case 0: ny = guardNode.gridY + 1; break; // Up
                case 1: ny = guardNode.gridY - 1; break; // Down
                case 2: nx = guardNode.gridX - 1; break; // Left
                case 3: nx = guardNode.gridX + 1; break; // Right
            }

            nx = Mathf.Clamp(nx, 0, gridManager.gridSizeX - 1);
            ny = Mathf.Clamp(ny, 0, gridManager.gridSizeY - 1);

            Vector3 worldPos = gridManager.GetWorldPosition(nx, ny);
            return worldPos;
        }
    }

    private int ArgMax(Tensor tensor)
    {
        float maxVal = float.MinValue;
        int maxIdx = 0;
        for (int i = 0; i < tensor.length; i++)
        {
            if (tensor[i] > maxVal)
            {
                maxVal = tensor[i];
                maxIdx = i;
            }
        }
        return maxIdx;
    }

    void OnDestroy()
    {
        if (worker != null)
            worker.Dispose();
    }
}

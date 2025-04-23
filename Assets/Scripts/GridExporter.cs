using UnityEngine;
using UnityEditor;
using System.IO;

public class GridExporterWindow : EditorWindow
{
    GridManager gridManager;
    string outputPath = "Assets/grid.json";

    [MenuItem("Tools/Export Grid")]
    public static void ShowWindow()
    {
        GetWindow<GridExporterWindow>("Export Grid");
    }

    void OnGUI()
    {
        GUILayout.Label("Grid JSON Exporter", EditorStyles.boldLabel);
        gridManager = (GridManager)EditorGUILayout.ObjectField("Grid Manager", gridManager, typeof(GridManager), true);
        outputPath = EditorGUILayout.TextField("Output Path", outputPath);

        if (GUILayout.Button("Export"))
        {
            if (gridManager == null)
            {
                Debug.LogError("Assign a GridManager first!");
                return;
            }
            ExportGrid();
        }
    }

    void ExportGrid()
    {
        int w = gridManager.gridSizeX;
        int h = gridManager.gridSizeY;
        bool[,] walkable = gridManager.GetWalkableArray(); 
        int[] cells = new int[w * h];
        for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            cells[y*w + x] = walkable[x, y] ? 1 : 0;

        var data = new GridData { width = w, height = h, cells = cells };
        string json = JsonUtility.ToJson(data, true);
        File.WriteAllText(outputPath, json);
        AssetDatabase.Refresh();
        Debug.Log($"Grid exported to {outputPath}");
    }

    [System.Serializable]
    class GridData
    {
        public int width;
        public int height;
        public int[] cells;
    }
}
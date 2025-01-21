using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
public class VolumeCalculator : MonoBehaviour
{
    [Header("Calculated Volume")]
     public float volume;

    void OnValidate()
    {
        CalculateAndAssignVolume();
    }

    void CalculateAndAssignVolume()
    {
        Mesh mesh = GetComponent<MeshFilter>().sharedMesh;
        if (mesh != null)
        {
            volume = CalculateMeshVolume(mesh);
        }
    }

    float CalculateMeshVolume(Mesh mesh)
    {
        float volume = 0;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 p1 = vertices[triangles[i]];
            Vector3 p2 = vertices[triangles[i + 1]];
            Vector3 p3 = vertices[triangles[i + 2]];
            volume += SignedVolumeOfTriangle(p1, p2, p3);
        }

        return Mathf.Abs(volume);
    }

    float SignedVolumeOfTriangle(Vector3 p1, Vector3 p2, Vector3 p3)
    {
        return Vector3.Dot(p1, Vector3.Cross(p2, p3)) / 6.0f;
    }
}

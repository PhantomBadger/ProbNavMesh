using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEditor;
using System;

public class GenerateMeshFromNavMesh : Editor
{
    /// <summary>
    /// Debug function to generate a mesh filter of the current nav mesh
    /// </summary>
    [MenuItem("Window/AI/Navigation/Generate Debug Mesh Filter")]
    public static void GenerateMeshFilter()
    {
        //Get the nav mesh triangulation data & call the extension method
        NavMeshTriangulation navMeshTriangulation = NavMesh.CalculateTriangulation();
        navMeshTriangulation.GenerateMeshFilterObject();
    }
}

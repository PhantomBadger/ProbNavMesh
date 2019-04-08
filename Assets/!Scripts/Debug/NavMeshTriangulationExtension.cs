using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public static class NavMeshTriangulationExtension
{
    /// <summary>
    /// Extension method to generate a debug game object containing a filter and renderer representing the nav mesh
    /// </summary>
    /// <returns>A GameObject containing a MeshFilter, MeshRenderer, and MeshCollider of the NavMesh</returns>
    public static GameObject GenerateMeshFilterObject(this NavMeshTriangulation navMeshTriangulation)
    {
        //Create a mesh to represent that
        Mesh mesh = GenerateMeshFilter(navMeshTriangulation);

        //Create a GameObject to be the base for our debugs
        GameObject go = new GameObject();

        //Add a MeshFilter and MeshRenderer component
        MeshFilter mf = go.AddComponent<MeshFilter>();
        MeshRenderer mr = go.AddComponent<MeshRenderer>();
        MeshCollider mc = go.AddComponent<MeshCollider>();

        //Assign our mesh
        mf.sharedMesh = mesh;
        mc.sharedMesh = mesh;

        //Name appropriately
        go.name = DateTime.Now.ToString("MMddhhmmss") + "_NavMesh";
        return go;
    }

    /// <summary>
    /// Extension method to generate a mesh from a NavMeshTriangulation instance
    /// </summary>
    /// <returns>A mesh instance representing the NavMesh</returns>
    public static Mesh GenerateMeshFilter(this NavMeshTriangulation navMeshTriangulation)
    {
        //Create a mesh to represent that
        Mesh mesh = new Mesh();
        mesh.vertices = navMeshTriangulation.vertices;
        mesh.triangles = navMeshTriangulation.indices;

        return mesh;
    }
}

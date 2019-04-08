using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

/// <summary>
/// Debug class used to test adjacent triangle detection, decoupled from Probability Layer, so functionality may be repeated from other classes
/// </summary>
public class TriangleSelection : MonoBehaviour
{
    /// <summary>
    /// A simple class representing a single Triangle
    /// </summary>
    private struct Triangle
    {
        public Vector3 v1;
        public Vector3 v2;
        public Vector3 v3;

        public Triangle(Vector3 newV1, Vector3 newV2, Vector3 newV3)
        {
            v1 = newV1;
            v2 = newV2;
            v3 = newV3;
        }
    }

    private List<Triangle> trianglesToDraw = new List<Triangle>();
    private bool drawTriangles = false;

    /// <summary>
    /// Called once per frame, highlights the triangle hit
    /// </summary>
    void Update()
    {
        trianglesToDraw.Clear();
        drawTriangles = false;

        if (Input.GetMouseButton(0))
        {
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out RaycastHit hit, 100))
            {
                //Check if the object has a collider
                MeshCollider meshCollider = hit.collider as MeshCollider;
                if (meshCollider == null || meshCollider.sharedMesh == null)
                {
                    return;
                }

                //Get the mesh, vertices, and triangles
                MeshFilter meshFilter = meshCollider.GetComponent<MeshFilter>();
                Mesh mesh = meshCollider.sharedMesh;

                Triangle verts = GetVerticesOfHitTriangle(hit.triangleIndex, mesh, meshCollider.transform);

                //Find triangle and neighbouring vertices
                trianglesToDraw.Add(verts);
                trianglesToDraw.AddRange(GetNeighbouringTriangles(hit.triangleIndex, mesh, meshCollider.transform));

                drawTriangles = true;
            }
        }
    }

    /// <summary>
    /// Gets a list of the neighbouring triangles
    /// </summary>
    /// <param name="hitIndex">The hit index to find the neighbours of</param>
    /// <param name="mesh">The mesh to search on</param>
    /// <param name="meshTransform">The mesh transform, used to turn the vertices to world space once found</param>
    /// <returns>A list of the Triangle struct containing all neighbouring triangles</returns>
    private List<Triangle> GetNeighbouringTriangles(int hitIndex, Mesh mesh, Transform meshTransform)
    {
        //Get the starting index of our area
        Vector3[] meshVertices = mesh.vertices;
        int[] meshTriangles = mesh.triangles;

        int indicesIndex = hitIndex * 3;

        //Get the local space vertices that were hit
        Vector3 v1 = meshVertices[meshTriangles[indicesIndex++]];
        Vector3 v2 = meshVertices[meshTriangles[indicesIndex++]];
        Vector3 v3 = meshVertices[meshTriangles[indicesIndex]];

        //We only treat it as a true neighbour if it is adjacent to two vertices
        //So we use a hashset to flag the first neighbouring vertex, then add it to the list
        //once the second is found
        HashSet<int> singlePassNeighbours = new HashSet<int>();
        List<int> doublePassNeighbours = new List<int>();

        //Go through the areas
        for (int i = 0; i < meshTriangles.Length; i++)
        {
            //Check if the current vertex matches one of the ones of our triangle
            Vector3 curVertex = meshVertices[meshTriangles[i]];
            if (curVertex == v1 || curVertex == v2 || curVertex == v3)
            {
                //If we havent registered it already, and the area isnt the one we're already using
                int neighbourIndex = i / 3;
                if (neighbourIndex == hitIndex)
                {
                    continue;
                }

                //If this is the first time we've seen it, add it to the single pass list
                if (!singlePassNeighbours.Contains(neighbourIndex))
                {
                    //Log it
                    singlePassNeighbours.Add(neighbourIndex);
                }
                //If we've seen it once already, then we can log it in the double pass and actually use it
                else if (singlePassNeighbours.Contains(neighbourIndex) && !doublePassNeighbours.Contains(neighbourIndex))
                {
                    doublePassNeighbours.Add(neighbourIndex);
                }
            }
        }

        List<Triangle> verts = new List<Triangle>();
        for (int i = 0; i < doublePassNeighbours.Count; i++)
        {
            verts.Add(GetVerticesOfHitTriangle(doublePassNeighbours[i], mesh, meshTransform));
        }
        return verts;
    }

    /// <summary>
    /// Get the vertices of the hit triangle
    /// </summary>
    /// <param name="hitTriangle">The index of the triangle hit on the mesh</param>
    /// <param name="mesh">The mesh to search on</param>
    /// <param name="meshTransform">The mesh transform, used to turn the vertices to world space once found</param>
    /// <returns>A Triangle instance representing the found triangle on the mesh</returns>
    private Triangle GetVerticesOfHitTriangle(int hitTriangle, Mesh mesh, Transform meshTransform)
    {
        Vector3[] meshVertices = mesh.vertices;
        int[] meshTriangles = mesh.triangles;

        //Get the local space vertices that were hit
        Vector3 v1 = meshVertices[meshTriangles[hitTriangle * 3]];
        Vector3 v2 = meshVertices[meshTriangles[(hitTriangle * 3) + 1]];
        Vector3 v3 = meshVertices[meshTriangles[(hitTriangle * 3) + 2]];

        //Transform local space vertices to world space
        v1 = meshTransform.TransformPoint(v1);
        v2 = meshTransform.TransformPoint(v2);
        v3 = meshTransform.TransformPoint(v3);

        return new Triangle(v1, v2, v3);
    }

    /// <summary>
    /// Called after rendering everything else, displays our debug triangle lines
    /// </summary>
    public void OnRenderObject()
    {
        if (drawTriangles)
        {
            //Draw the triangle as a series of lines
            for (int i = 0; i < trianglesToDraw.Count; i++)
            {
                GL.PushMatrix();
                GL.Begin(GL.LINES);
                GL.Color(Color.green);
                GL.Vertex(trianglesToDraw[i].v1);
                GL.Vertex(trianglesToDraw[i].v2);
                GL.Vertex(trianglesToDraw[i].v2);
                GL.Vertex(trianglesToDraw[i].v3);
                GL.Vertex(trianglesToDraw[i].v3);
                GL.Vertex(trianglesToDraw[i].v1);
                GL.End();
                GL.PopMatrix();
            }
        }
    }
}

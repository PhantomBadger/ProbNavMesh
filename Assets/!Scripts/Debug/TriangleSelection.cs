using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace ProbabilityNavMesh
{
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

        public string DebugTag = "DebugNavMesh";
        private List<Triangle> trianglesToDraw = new List<Triangle>();
        private bool drawTriangles = false;
        private ProbabilityNavMesh.ProbNavMeshTriangulation probNavMeshTriangulation;

        /// <summary>
        /// Called at the Start, initializes the Probability Layer of the NavMesh
        /// </summary>
        private void Start()
        {
            NavMeshTriangulation nmt = NavMesh.CalculateTriangulation();
            probNavMeshTriangulation = new ProbabilityNavMesh.ProbNavMeshTriangulation(nmt);
        }

        /// <summary>
        /// Called once per frame, highlights the triangle hit
        /// </summary>
        private void Update()
        {
            trianglesToDraw.Clear();
            drawTriangles = false;

            if (Input.GetMouseButton(0))
            {
                //We use the debug mesh to evaluate where our raycast hit
                if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out RaycastHit hit, 100))
                {
                    //Check if the object has a collider
                    MeshCollider meshCollider = hit.collider as MeshCollider;
                    if (meshCollider == null || meshCollider.sharedMesh == null || hit.collider.tag != DebugTag)
                    {
                        return;
                    }

                    //Then we use the NavMeshTriangulationData for everything else
                    Vector3[] meshVerts = probNavMeshTriangulation.NavMeshTriangulationData.vertices;
                    int[] meshTriangles = probNavMeshTriangulation.NavMeshTriangulationData.indices;

                    //Find triangle where we hit
                    Vector3[] middleVerts = probNavMeshTriangulation.GetVerticesOfTriangle(hit.triangleIndex, meshVerts, meshTriangles);
                    Triangle middleTri = new Triangle(middleVerts[0], middleVerts[1], middleVerts[2]);
                    trianglesToDraw.Add(middleTri);

                    //Find the vertices of the neighbouring triangles
                    int[] neighbouringIndices = probNavMeshTriangulation.GetNeighboursOfTriangle(hit.triangleIndex, meshVerts, meshTriangles);
                    for (int i = 0; i < neighbouringIndices.Length; i++)
                    {
                        Vector3[] neighbourVerts = probNavMeshTriangulation.GetVerticesOfTriangle(neighbouringIndices[i], meshVerts, meshTriangles);
                        Triangle neighbourTri = new Triangle(neighbourVerts[0], neighbourVerts[1], neighbourVerts[2]);
                        trianglesToDraw.Add(neighbourTri);
                    }

                    drawTriangles = true;
                }
            }
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
}

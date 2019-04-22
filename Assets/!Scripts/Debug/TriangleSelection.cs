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

        [Header("Propagator")]
        public ProbabilityPropagator Propagator;
        
        [Header("Debug Settings")]
        public string DebugTag = "DebugNavMesh";
        public Material DebugMaterial; 
        public Color LowProbabilityColor = Color.gray;
        public Color HighProbabilityColor = Color.green;

        private List<Triangle> trianglesToDraw = new List<Triangle>();
        private ProbNavMeshTriangulation probNavMeshTriangulation;
        private bool drawTriangles = false;

        private Vector3[] meshVerts;
        private int[] meshTriangles;

        /// <summary>
        /// Called at the start, populates our internal array of mesh data
        /// </summary>
        private void Start()
        {
            probNavMeshTriangulation = Propagator.GetProbabilityNavMeshTriangulation();

            meshVerts = probNavMeshTriangulation.NavMeshTriangulationData.vertices;
            meshTriangles = probNavMeshTriangulation.NavMeshTriangulationData.indices;
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
                int triangleIndex = 0;

                //We use the debug mesh to evaluate where our raycast hit
                if (GetClickedOnTriangle(ref triangleIndex))
                {
                    //Then we use the NavMeshTriangulationData for everything else
                    //Find triangle where we hit
                    Vector3[] middleVerts = probNavMeshTriangulation.GetVerticesOfTriangle(triangleIndex, meshVerts, meshTriangles);
                    Triangle middleTri = new Triangle(middleVerts[0], middleVerts[1], middleVerts[2]);
                    trianglesToDraw.Add(middleTri);

                    //Find the vertices of the neighbouring triangles
                    int[] neighbouringIndices = probNavMeshTriangulation.GetNeighboursOfTriangle(triangleIndex, meshVerts, meshTriangles);
                    for (int i = 0; i < neighbouringIndices.Length; i++)
                    {
                        Vector3[] neighbourVerts = probNavMeshTriangulation.GetVerticesOfTriangle(neighbouringIndices[i], meshVerts, meshTriangles);
                        Triangle neighbourTri = new Triangle(neighbourVerts[0], neighbourVerts[1], neighbourVerts[2]);
                        trianglesToDraw.Add(neighbourTri);
                    }

                    drawTriangles = true;
                }
            }
            else if (Input.GetMouseButtonDown(1))
            {
                int triangleIndex = 0;

                //We use the debug mesh to evaluate where our raycast hit
                if (GetClickedOnTriangle(ref triangleIndex))
                {
                    //Set the probability of the selected triangle
                    Propagator.SetAllProbability(0);
                    Propagator.SetProbability(triangleIndex, 1);
                    Propagator.StartPropagation();
                }
            }
        }

        /// <summary>
        /// Gets the index of the triangle clicked on by the user
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle clicked on, -1 if nothing was clicked on</param>
        /// <returns>True if the mesh was clicked on, false if it wasn't</returns>
        public bool GetClickedOnTriangle(ref int triangleIndex)
        {
            triangleIndex = -1;

            //We use the debug mesh to evaluate where our raycast hit
            if (Physics.Raycast(Camera.main.ScreenPointToRay(Input.mousePosition), out RaycastHit hit, 100))
            {
                //Check if the object has a collider
                MeshCollider meshCollider = hit.collider as MeshCollider;
                if (meshCollider == null || meshCollider.sharedMesh == null || hit.collider.tag != DebugTag)
                {
                    return false;
                }

                triangleIndex = hit.triangleIndex;
                return true;
            }
            return false;
        }

        /// <summary>
        /// Called after rendering everything else, displays our debug triangle lines
        /// </summary>
        public void OnRenderObject()
        {
            string debugText = "";

            //Draw the probability of the triangles
            for (int i = 0; i < meshTriangles.Length - 1; i+=3)
            {
                int triangleIndex = i / 3;
                debugText += "\nIndex: " + i + " Prob: " + probNavMeshTriangulation.Probability[triangleIndex];
                Vector3 v1, v2, v3;
                v1 = meshVerts[meshTriangles[i+0]];
                v2 = meshVerts[meshTriangles[i+1]];
                v3 = meshVerts[meshTriangles[i+2]];

                //Draw the triangle
                GL.PushMatrix();
                DebugMaterial.SetPass(0);
                GL.Begin(GL.TRIANGLES);

                //Set the colour to the set probability
                Color newCol = Color.Lerp(LowProbabilityColor, HighProbabilityColor, probNavMeshTriangulation.Probability[triangleIndex]);
                debugText += " Color: " + newCol.ToString();
                GL.Color(newCol);

                GL.Vertex(v1);
                GL.Vertex(v2);
                GL.Vertex(v3);

                GL.End();
                GL.PopMatrix();
            }
            Debug.Log(debugText);

            //If neighbours are selected, draw them
            if (drawTriangles)
            {
                //Draw the triangle as a series of lines
                for (int i = 0; i < trianglesToDraw.Count; i++)
                {
                    GL.PushMatrix();
                    GL.Begin(GL.LINES);
                    GL.Color(Color.white);
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

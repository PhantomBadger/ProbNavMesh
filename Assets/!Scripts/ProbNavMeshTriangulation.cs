using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace ProbabilityNavMesh
{
    public class ProbNavMeshTriangulation
    {
        public NavMeshTriangulation NavMeshTriangulationData;
        public float[] Probability;

        /// <summary>
        /// Constructor which takes in an existing NavMeshTriangulation and adds a probability layer to it
        /// </summary>
        /// <param name="navMeshTriangulation">The NavMeshTriangulation struct instance to use</param>
        public ProbNavMeshTriangulation(NavMeshTriangulation navMeshTriangulation)
        {
            NavMeshTriangulationData = navMeshTriangulation;
            Probability = new float[NavMeshTriangulationData.indices.Length / 3];
        }

        /// <summary>
        /// Gets the index of the triangle the point is within
        /// </summary>
        /// <param name="points">The point to evaluate</param>
        /// <remarks>If there are no triangles in the NavMesh, it will return -1. This comparison is purely 2D, and ignores the height of the point</remarks>
        /// <returns>The index of the closest triangle, -1 for invalid data.</returns>
        public int EvaluatePoint(Vector3 point)
        {
            int closestTriangleIndex = -1;

            if (point == null)
            {
                return closestTriangleIndex;
            }

            for (int i = 0; i < NavMeshTriangulationData.indices.Length - 2; i += 3)
            {
                // Get triangle verts
                Vector3 v1 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i]];
                Vector3 v2 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i + 1]];
                Vector3 v3 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i + 2]];

                /*=============*/
                // Snippet taken from https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
                var s = v1.z * v3.x - v1.x * v3.z + (v3.z - v1.z) * point.x + (v1.x - v3.x) * point.z;
                var t = v1.x * v2.z - v1.z * v2.x + (v1.z - v2.z) * point.x + (v2.x - v1.x) * point.z;

                if ((s < 0) != (t < 0))
                {
                    continue;
                }

                var A = -v2.z * v3.x + v1.z * (v3.x - v2.x) + v1.x * (v2.z - v3.z) + v2.x * v3.z;

                bool isInside =  A < 0 ?
                                (s <= 0 && s + t >= A) :
                                (s >= 0 && s + t <= A);
                /*=============*/

                if (isInside)
                {
                    closestTriangleIndex = i / 3;
                    break;
                }
            }

            return closestTriangleIndex;
        }

        /// <summary>
        /// Gets a list of indices of neighbouring triangles
        /// </summary>
        /// <param name="triangleIndex">The index of the target triangle</param>
        /// <param name="meshVertices">An array of vertices from the target mesh</param>
        /// <param name="meshTriangles">An array of triangle indices from the target mesh</param>
        /// <returns>An array of integers representing the index of the neighbouring triangles</returns>
        public int[] GetNeighboursOfTriangle(int triangleIndex, Vector3[] meshVertices, int[] meshTriangles)
        {
            int indicesIndex = triangleIndex * 3;

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
                    if (neighbourIndex == triangleIndex)
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

            return doublePassNeighbours.ToArray();
        }

        /// <summary>
        /// Gets an array of the vertices used for a given triangle index.
        /// </summary>
        /// <param name="triangleIndex">The triangle index to use</param>
        /// <param name="meshVertices">The vertices of the target mesh</param>
        /// <param name="meshTriangles">The triangle indices of the target mesh</param>
        /// <returns>An array of three Vector3 elements modelling the triangle's vertices</returns>
        public Vector3[] GetVerticesOfTriangle(int triangleIndex, Vector3[] meshVertices, int[] meshTriangles)
        {
            int indicesIndex = triangleIndex * 3;

            //Get the local space vertices that were hit
            Vector3 v1 = meshVertices[meshTriangles[indicesIndex++]];
            Vector3 v2 = meshVertices[meshTriangles[indicesIndex++]];
            Vector3 v3 = meshVertices[meshTriangles[indicesIndex]];

            return new Vector3[] { v1, v2, v3 };
        }
    }
}

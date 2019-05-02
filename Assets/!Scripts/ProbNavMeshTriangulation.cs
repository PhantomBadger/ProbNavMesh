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
            int closestIndex = -1;
            float closestDistance = float.MaxValue;

            if (point == null)
            {
                return closestIndex;
            }

            for (int i = 0; i < NavMeshTriangulationData.indices.Length - 2; i += 3)
            {
                // Get triangle verts
                Vector3 v1 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i]];
                Vector3 v2 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i + 1]];
                Vector3 v3 = NavMeshTriangulationData.vertices[NavMeshTriangulationData.indices[i + 2]];

                // Calculate the normal of the triangle
                Vector3 triangleNormal = Vector3.Cross((v2 - v1).normalized, (v3 - v1).normalized);

                // Project our test point onto the plane
                Vector3 projectedPoint = point + Vector3.Dot((v1 - point), triangleNormal) * triangleNormal;

                // Calculate the Barycentric coordinates of the point relative to our triangle
                /*======*/
                // Snippet adapted from https://answers.unity.com/questions/424974/nearest-point-on-mesh.html
                var u = ((projectedPoint.x * v2.y) - (projectedPoint.x * v3.y) - (v2.x * projectedPointPoint.y) + (v2.x * v3.y) + (v3.x * projectedPoint.y) - (v3.x * v2.y)) /
                        ((v1.x * v2.y) - (v1.x * v3.y) - (v2.x * v1.y) + (v2.x * v3.y) + (v3.x * v1.y) - (v3.x * v2.y));
                var v = ((v1.x * projectedPoint.y) - (v1.x * v3.y) - (projectedPoint.x * v1.y) + (projectedPoint.x * v3.y) + (v3.x * v1.y) - (v3.x * projectedPoint.y)) /
                        ((v1.x * v2.y) - (v1.x * v3.y) - (v2.x * v1.y) + (v2.x * v3.y) + (v3.x * v1.y) - (v3.x * v2.y));
                var w = ((v1.x * v2.y) - (v1.x * projectedPoint.y) - (v2.x * v1.y) + (v2.x * projectedPoint.y) + (projectedPoint.x * v1.y) - (projectedPoint.x * v2.y)) /
                        ((v1.x * v2.y) - (v1.x * v3.y) - (v2.x * v1.y) + (v2.x * v3.y) + (v3.x * v1.y) - (v3.x * v2.y));
                /*======*/

                // Compose the nearest point
                // The u, v, w vector is a percentage of v1, v2, and v3, so we have to times them back out 
                // to get the actual point.
                Vector3 composedBarycentric = new Vector3(u, v, w);
                composedBarycentric.Normalize();
                Vector3 nearestPoint = (v1 * composedBarycentric.x) + 
                                       (v2 * composedBarycentric.y) + 
                                       (v3 * composedBarycentric.z);

                // Compare the distance
                float distance = Vector3.Distance(nearestPoint, point);
                if (distance < closestDistance)
                {
                    closestIndex = i;
                    closestDistance = distance;
                }
            }

            return closestIndex;
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

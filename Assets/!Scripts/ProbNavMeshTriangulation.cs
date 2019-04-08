tyusing System.Collections;
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
            Probability = new float[NavMeshTriangulationData.indices / 3];
        }

        /// <summary>
        /// Returns the indices of the neighboring areas in the triangulation data 
        /// </summary>
        /// <param name="areaIndex">The index of the target area in the triangulation data</param>
        /// <returns>An array of indices of areas in the triangulation data</returns>
        public static int[] GetNeighbors(int areaIndex, NavMeshTriangulation navMeshTriangulation)
        {
            //Get the starting index of our area
            int indicesIndex = areaIndex * 3;

            //Get the local space vertices that were hit
            Vector3 v1 = navMeshTriangulation.vertices[navMeshTriangulation.indices[indicesIndex++]];
            Vector3 v2 = navMeshTriangulation.vertices[navMeshTriangulation.indices[indicesIndex++]];
            Vector3 v3 = navMeshTriangulation.vertices[navMeshTriangulation.indices[indicesIndex]];

            //We only treat it as a true neighbour if it is adjacent to two vertices
            //So we use a hashset to flag the first neighbouring vertex, then add it to the list
            //once the second is found
            HashSet<int> singlePassNeighbours = new HashSet<int>();
            List<int> doublePassNeighbours = new List<int>();

            //Go through the areas
            for (int i = 0; i < navMeshTriangulation.indices.Length; i++)
            {
                //Check if the current vertex matches one of the ones of our triangle
                Vector3 curVertex = navMeshTriangulation.vertices[navMeshTriangulation.indices[i]];
                if (curVertex == v1 || curVertex == v2 || curVertex == v3)
                {
                    //If we havent registered it already, and the area isnt the one we're already using
                    int neighbourIndex = i / 3;
                    if (neighbourIndex == areaIndex)
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

            //Return the found neighbours
            return doublePassNeighbours.ToArray();
        }
    }
}

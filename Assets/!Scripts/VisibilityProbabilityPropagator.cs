using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace ProbabilityNavMesh
{
    /// <summary>
    /// This class should better mimic how an Occupancy Grid is intended to work.
    /// It will take the previous map state, identify which cells are currently observed
    /// Then propagate the probability to unobserved adjacent cells
    /// </summary>
    public class VisibilityProbabilityPropagator : ProbabilityPropagator
    {
        public HashSet<int> CurrentlyObservedTriangles = new HashSet<int>();

        /// <summary>
        /// Adds the specified triangle index to the set of observed triangles.
        /// If the triangle is already observed, it will do nothing.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle being observed</param>
        public void MarkTriangleAsObserved(int triangleIndex)
        {
            if (!CurrentlyObservedTriangles.Contains(triangleIndex))
            {
                CurrentlyObservedTriangles.Add(triangleIndex);
            }
        }

        /// <summary>
        /// Removes the specified triangle index from the set of observed triangles.
        /// If the triangle is already not observed, it will do nothing.
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle no longer being observed</param>
        public void MarkTriangleAsUnobserved(int triangleIndex)
        {
            if (CurrentlyObservedTriangles.Contains(triangleIndex))
            {
                CurrentlyObservedTriangles.Remove(triangleIndex);
            }
        }

        /// <summary>
        /// Returns whether the specified triangle index is in the observed list or not.
        /// </summary>
        /// <returns>True if it is in the list, false if it isn't</returns>
        public bool IsTriangleObserved(int triangleIndex)
        {
            return CurrentlyObservedTriangles.Contains(triangleIndex);
        }

        /// <summary>
        /// Propagates the probability to adjacent triangles. Once a probability goes over 0.5f it cannot go below it.
        /// </summary>
        protected override void PropagateProbability()
        {
            float[] probChanges = new float[probabilityNavMesh.Probability.Length];
            string debugText = "Propagating Probability";

            //Iterate over all triangles (same indices as probability)
            for (int i = 0; i < probabilityNavMesh.Probability.Length; i++)
            {
                debugText += "\nPropagating " + i;
                //If this cell is currently observed, skip it
                if (IsTriangleObserved(i))
                {
                    debugText += "\n\tTriangle " + i + " is observed";
                    continue;
                }

                //Get our probability
                float curProb = probabilityNavMesh.Probability[i];
                debugText += "\n\tCurrentTriangleProb: " + curProb;

                //If the probability is less than our minimum, don't bother propagating it
                if (curProb < MIN_PROBABILITY)
                {
                    continue;
                }

                //Get the indices of the adjacent triangles
                int[] neighbourTriangles = probabilityNavMesh.GetNeighboursOfTriangle(i, probabilityNavMesh.NavMeshTriangulationData.vertices, probabilityNavMesh.NavMeshTriangulationData.indices);
                //Limit the neighbours to only those currently being Unobserved
                neighbourTriangles = neighbourTriangles.Where(neighbourIndex => !IsTriangleObserved(neighbourIndex)).ToArray();
                debugText += "\n\tNumber of Neighbours: " + neighbourTriangles.Length;
                for (int j = 0; j < neighbourTriangles.Length; j++)
                {
                    debugText += "\n\tEvaluating Neighbour " + j;
                    //We attempt to find a proportion of excess probability between our node and the neighbour node
                    //And use this to calculate how much is moved

                    //Get the neighbouring cell's probability
                    float neighbourProb = probabilityNavMesh.Probability[neighbourTriangles[j]];

                    //Find the rate at which our probability will flow into it
                    //  1 - neighbourProb           Gets us an inverted value of the probability, meaning a lower value is a higher flow rate
                    //                    * 0.5f    Since 0.5f is our target value, this ensures that our normalized value is between 0 and 0.5f. 
                    //                              It ensures that we don't 'give away' too much probability at a time.
                    float flowRate = (1 - neighbourProb) * 0.5f;

                    //Find the delta probability between thw two cells and multiply it by the flow rate
                    //The Mathf.Max function ensures that if the neighbouring cell has a higher probability than us, we don't change it, as we use 0
                    float deltaProb = Mathf.Max(0, curProb - neighbourProb) * flowRate;

                    //Divide the deltaProb by the number of neighbours to represent the probability moving equally amongst it's neighbouring nodes
                    //(Even if less is actually sent to one due to the flowRate)
                    deltaProb /= neighbourTriangles.Length;

                    //Increment the probability change array
                    probChanges[neighbourTriangles[j]] += deltaProb;

                    debugText += "\n\t\tNeighbour " + j + " new Prob: " + probChanges[neighbourTriangles[j]];

                    //If lowering the probability would not put it below 0.5f, our target, then we can lower our probability too.
                    //TODO: Change this to be a little more elegant..
                    if ((curProb - (probChanges[i] - deltaProb)) > 0.5f)
                    {
                        probChanges[i] -= deltaProb;
                    }
                }
                debugText += "\n\tNew Prob: " + probChanges[i];
            }

            //Update our probability array with the new changes
            for (int i = 0; i < probabilityNavMesh.Probability.Length && i < probChanges.Length; i++)
            {
                probabilityNavMesh.Probability[i] = Mathf.Clamp01(probabilityNavMesh.Probability[i] + probChanges[i]);
            }
        }
    }
}

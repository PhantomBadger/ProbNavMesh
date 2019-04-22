using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using System.Linq;

namespace ProbabilityNavMesh
{
    public class ProbabilityPropagator : MonoBehaviour
    {
        [Tooltip("How many seconds between each propagation 'tick'. If zero it will never propagate")]
        public float PropagationSpeedInSeconds = 1.0f;

        private ProbNavMeshTriangulation probabilityNavMesh;
        private float propagationCounter = 0.0f;
        private bool shouldBePropagating = false;

        private const float MIN_PROBABILITY = 0.05f;

        /// <summary>
        /// Called at the Start, initializes the Probability Layer
        /// </summary>
        private void Start()
        {
            NavMeshTriangulation nmt = NavMesh.CalculateTriangulation();
            probabilityNavMesh = new ProbNavMeshTriangulation(nmt);
        }

        /// <summary>
        /// Called every frame, calls the propagation method at the desired interval
        /// </summary>
        private void Update()
        {
            //If we have waited long enough
            if (shouldBePropagating && ((propagationCounter += Time.deltaTime) > PropagationSpeedInSeconds))
            {
                //Loop around, this should help mitigate drifting due to varying deltaTimes
                propagationCounter %= PropagationSpeedInSeconds;

                //Propagate probability across the mesh
                PropagateProbability();
            }
        }

        /// <summary>
        /// Resets the probability of all triangles in the Nav Mesh to an equal probability of being occupied or not
        /// </summary>
        public void ResetProbability()
        {
            SetAllProbability(0.5f);
        }

        /// <summary>
        /// Sets the probabiltiy of all triangles to the desired value
        /// </summary>
        /// <param name="newProbability">The normalizes value to set every triangle to. Clamped between 0 and 1</param>
        public void SetAllProbability(float newProbability)
        {
            newProbability = Mathf.Clamp01(newProbability);
            for (int i = 0; i < probabilityNavMesh.Probability.Length; i++)
            {
                probabilityNavMesh.Probability[i] = newProbability;
            }
        }

        /// <summary>
        /// Sets the probability of a given triangle to a desired value
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle to set, will be multiplied by 3 to get the values in the corresponding indices array</param>
        /// <param name="newProbability">The probability to set the triangle to. Clamped to be between zero and one.</param>
        public void SetProbability(int triangleIndex, float newProbability)
        {
            newProbability = Mathf.Clamp01(newProbability);
            if (triangleIndex < probabilityNavMesh.Probability.Length)
            {
                probabilityNavMesh.Probability[triangleIndex] = newProbability;
            }
            else
            {
                Debug.LogError("ERROR: Invalid Triangle Index: " + triangleIndex + " supplied! Must be between 0 and " + (probabilityNavMesh.Probability.Length - 1));
            }
        }

        /// <summary>
        /// Gets the probability of a given triangle. If an invalid index is given, -1 will be returned instead
        /// </summary>
        /// <param name="triangleIndex">The index of the triangle to set, will be multiplied by 3 to get the values in the corresponding indices array</param>
        /// <returns>The probability value of that triangle, between 0 and 1</returns>
        public float GetProbability(int triangleIndex)
        {
            if (triangleIndex < probabilityNavMesh.Probability.Length)
            {
                return probabilityNavMesh.Probability[triangleIndex];
            }
            else
            {
                Debug.LogError("ERROR: Invalid Triangle Index: " + triangleIndex + " supplied! Must be between 0 and " + (probabilityNavMesh.Probability.Length - 1));
                return -1;
            }
        }

        /// <summary>
        /// Starts the propagation process
        /// </summary>
        public void StartPropagation()
        {
            shouldBePropagating = true;
        }

        /// <summary>
        /// Stops the propagation process
        /// </summary>
        public void StopPropagation()
        {
            shouldBePropagating = false;
        }

        /// <summary>
        /// Gets a reference to the probability nav mesh triangulation
        /// </summary>
        public ProbNavMeshTriangulation GetProbabilityNavMeshTriangulation()
        {
            return probabilityNavMesh;
        }

        /// <summary>
        /// Gets the index of the triangle with the highest probability.
        /// If there are no elements in the probability layer, it returns -1 instead.
        /// </summary>
        public int GetHighestProbabilityIndex()
        {
            int index = -1;
            float curHighestProb = float.MinValue;

            //Iterate over each probability
            for (int i = 0; i < probabilityNavMesh.Probability.Length; i++)
            {
                //If we havent recorded a value yet, or the new value is higher, replace it
                float prob = probabilityNavMesh.Probability[i];
                if (index < 0 || prob > curHighestProb)
                {
                    index = i;
                    curHighestProb = prob;
                }
            }
            return index;
        }

        /// <summary>
        /// Propagates the probability to adjacent triangles
        /// </summary>
        private void PropagateProbability()
        {
            float[] probChanges = new float[probabilityNavMesh.Probability.Length];

            //Iterate over all triangles (same indices as probability)
            for (int i = 0; i < probabilityNavMesh.Probability.Length; i++)
            {
                //Get our probability
                float curProb = probabilityNavMesh.Probability[i];

                //If the probability is less than our minimum, don't bother propagating it
                if (curProb < MIN_PROBABILITY)
                {
                    continue;
                }

                //Get the indices of the adjacent triangles
                int[] neighbourTriangles = probabilityNavMesh.GetNeighboursOfTriangle(i, probabilityNavMesh.NavMeshTriangulationData.vertices, probabilityNavMesh.NavMeshTriangulationData.indices);

                for (int j = 0; j < neighbourTriangles.Length; j++)
                {
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

                    //If lowering the probability would not put it below 0.5f, our target, then we can lower our probability too.
                    //TODO: Change this to be a little more elegant..
                    if ((curProb - (probChanges[i] - deltaProb)) > 0.5f)
                    {
                        probChanges[i] -= deltaProb;
                    }
                }
            }

            //Update our probability array with the new changes
            for (int i = 0; i < probabilityNavMesh.Probability.Length && i < probChanges.Length; i++)
            {
                probabilityNavMesh.Probability[i] = Mathf.Clamp01(probabilityNavMesh.Probability[i] + probChanges[i]);
            }
        }

        //=================================================================================================
        //Notes on Bayes Algorithm, Log-Odd, and Occupancy Grids, a good place for this system to evolve to
        //=================================================================================================
        //
        // z = measurement z~{0, 1}
        //  0 free, 1 occupied
        // p(z|Mxy) = data model
        //
        // Possible outcomes are
        // p(z = 1|Mxy = 1) = True Measurement of Occupied
        // p(z = 0|MXy = 1) = False Measurement of Free
        // p(z = 1|Mxy = 0) = False Measurement of Occupied
        // p(z = 0|MXy = 0) = True Measurement of Free
        //
        // P(Ac|B) = 1 - P(A|B)
        //
        //Prior Map = p(Mxy)
        //When used with Measurement Model p(z|Mxy)
        //Give us the Posterior Map p(Mxy|z)
        //
        //Baye's Rule:
        //Posterior     Likelihood  Prior       Evidence
        //p(Mxy|z) =    (p(z|Mxy)   p(Mxy)) /   p(z)
        //
        //In Bayesian Statistics the Posterior Probability of a random event or an uncertain proposition is the 
        //conditional probability that is assigned after the rleevant evidence or background is taken into account
        //
        //If there is a probability of something happening as p(X), the odds can be considered a ratio, and modelled as such:
        //Odd:= X Happens / X not Happens = p(X) / p(Xc)
        //
        //If we use the odds of whether a cell is occupied
        //Odd((Mxy=1) given z) = p(Mxy = 1|z) / p(Mxy = 0|z)
        //
        //Bayes Rule shows that
        //p(Mxy = 1|z) = (p(z|Mxy = 1)p(Mxy = 1)) / p(z)
        //p(Mxy = 0|z) = (p(z|Mxy = 0)p(Mxy = 0)) / p(z)
        //
        //So we can change our equation to
        //Odd = (p(z|Mxy = 1)p(Mxy = 1)/p(z)) / ((p(z|Mxy = 0)p(Mxy = 0)) / p(z))
        //Odd = (p(z|Mxy = 1)p(Mxy = 1)) / (p(z|Mxy = 0)p(Mxy = 0))
        //          p(Mxy = 1|z)   (p(z|Mxy = 1)p(Mxy = 1))
        // Odd:     ------------ = -----------------------
        //          p(Mxy = 0|z)   (p(z|Mxy = 0)p(Mxy = 0))
        //
        //If we take the logarithm of the odds it gets simpler (apparently???)
        //                  p(Mxy = 1|z)        (p(z|Mxy = 1)p(Mxy = 1))
        // Log-Odd:     log ------------ =  log -----------------------
        //                  p(Mxy = 0|z)        (p(z|Mxy = 0)p(Mxy = 0))
        //Which due to the properties of logarithmic equations can be simplified to
        //                  p(Mxy = 1|z)         p(z|Mxy = 1)        p(Mxy = 1)
        // Log-Odd:     log ------------ =  log ------------- + log --------
        //                  p(Mxy = 0|z)         p(z|Mxy = 0)        p(Mxy = 0)
        // Log odd+ = log odd meas + log odd-
        //The prior map stores log-odd values, the measurement model provides log-odd-meas, updating the posterior map with log-odd
        //Map updates becomes simply additions of those log-odd values
        //Update is only done for observed cells, the updated values become priors when you receive new measurements for future time steps
        //
        //Measurement model in log-odd form
        //     p(z|Mxy = 1)
        // log ------------
        //     p(z|Mxy = 0)
        //Two possible measurements:
        // Case 1: Cells with z = 1
        // Case 2: Cells with z = 0
        // Trivial Case: cells not measured
        //
        //Case 1:
        //                          p(z = 1|Mxy = 1)
        // log-odd-occupied := log -----------------
        //                          p(z = 1|Mxy = 0)
        //
        //Case 2:
        //                          p(z = 0|Mxy = 0)
        // log-odd-free     := log -----------------
        //                          p(z = 0|Mxy = 1)
        // Note that the Mxy values are flipped to show that Mxy = 0 matches with z = 0
        //
        //Update Rule: log-odd += log-odd-meas
        //---------------
        //Example:
        //---------------
        // Copnstant Measurement Model:
        // log-odd-occ-param := 0.9
        // log-odd-free-param := 0.7
        // The initial map is log-odd = 0 for all xy. Which is the equivalent of each cell having the samr probability of being occupied or free (0.5)
        // We measure infront of us, the cells that are observed to be occupied get the log-odd-occ-param value set to them, For cells observed to be free
        // we subtract the log-odd-free-param, giving the occupied cell a log-odd of 0.9, and the free cells a log-odd of -0.7
        // When we receive a new measurement, we repeat, so the more we observe the higher the probability that we have that section correct.
    }
}

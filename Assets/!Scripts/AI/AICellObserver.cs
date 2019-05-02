using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.AI
{

    /// <summary>
    /// A simple AI Agent that will flag observed cells to the VisibilityProbabilityPropagator.
    /// Gets observed triangles using a Ray-Marching system
    /// </summary>
    public class AICellObserver : MonoBehaviour
    {
        [Header("Propagator")]
        public VisibilityProbabilityPropagator Propagator;

        [Header("Ray-March Settings")]
        [Tooltip("The field of vision the AI will look between in degrees. The mid-point of this range will be aligned with the forward vector of this transform")]
        public float FieldOfVision = 90;
        [Tooltip("The number of rays marched out in a circle, It will determine the angle between them via FieldOfVision/NumberOfRays")]
        public int NumberOfRays = 8;
        [Tooltip("The maximum points along the ray it will try. This, combined with MarchDistance models the AIs 'vision'")]
        public int NumberOfMarches = 10;
        [Tooltip("The difference between each evaluation point on the march")]
        public float MarchInterval = 0.1f;
        [Space]
        [Tooltip("The amount of time, in seconds, between each complete ray march is performed. Note that this is not the time between each individual march, but instead the time between a complete observation.")]
        public float TimeBetweenEachObservationInSeconds = 0.5f;
        [Tooltip("The origin position (local to this object) that the Ray-March will use as it's origin.")]
        public Vector3 RayMarchOriginPosition = Vector3.zero;
        [Tooltip("The LayerMask used by the RayMarch to know whether it should stop prematurely (if it hits a wall, etc)")]
        public LayerMask RayMarchLayerMask = Physics.AllLayers;

        [Header("Line Of Sight")]
        [Tooltip("The max distance something can be seen at")]
        public float MaxLOSDistance = 15f;
        [Tooltip("The LayerMask used during the line of sight raycast test")]
        public LayerMask VisibilityLayerMask = 0;

        [Header("Debug")]
        public bool ShowDebug = false;

        protected float observationCounter = 0.0f;
        protected bool valid = true;

        /// <summary>
        /// Called at the start, tries to repair missing references, and if it fails it reports an error
        /// </summary>
        protected void Start()
        {
            //If we dont have a reference to the propagator, try to get it ourselves
            if (!Propagator)
            {
                Propagator = GameObject.FindObjectOfType<VisibilityProbabilityPropagator>();
            }

            //If we still dont have a reference, report an error
            if (!Propagator)
            {
                Debug.LogError("ERROR: No Propagator found! Unable to continue.");
                valid = false;
            }
        }

        /// <summary>
        /// Called once per frame, calls the Ray-Marching methods at the correct intervals
        /// </summary>
        protected void Update()
        {
            //If this instance isn't valid, don't perform any checks
            if (!valid)
            {
                return;
            }

            //If the observation counter has met our time goal, observe the surroundings
            if ((observationCounter += Time.deltaTime) > TimeBetweenEachObservationInSeconds)
            {
                observationCounter = 0f;
                ObserveCells();
            }
        }

        /// <summary>
        /// Observes the surrounding cells using a Ray-March and flags them as observed
        /// </summary>
        protected void ObserveCells()
        {
            float rayMarchDegreeInterval = FieldOfVision / NumberOfRays;
            float halfFOV = FieldOfVision / 2f;

            //Get the initial direction of the first Ray March
            Vector3 relativeDirection = transform.forward;
            relativeDirection = Quaternion.AngleAxis(halfFOV, Vector3.down) * relativeDirection;

            Vector3 worldSpaceOrigin = transform.localToWorldMatrix * RayMarchOriginPosition;

            //Go over each ray we intend to march across, updating the relative direction appropriately
            for (int i = 0; i < NumberOfRays; i++, relativeDirection = Quaternion.AngleAxis(rayMarchDegreeInterval, Vector3.up) * relativeDirection)
            {
                Ray ray = new Ray()
                {
                    origin = worldSpaceOrigin,
                    direction = relativeDirection.normalized
                };

                float maxDistance = MarchInterval * NumberOfMarches;

                if (Physics.Raycast(ray, out RaycastHit hitInfo, RayMarchLayerMask))
                {
                    maxDistance = Vector3.Distance(hitInfo.point, ray.origin);
                }

                //Find observed cells
                int[] observedCells = RayMarch(ray, MarchInterval, NumberOfMarches, maxDistance);

                //Mark each cell as observed in the NavMesh
                for (int j = 0; j < observedCells.Length; j++)
                {
                    if (!Propagator.IsTriangleObserved(observedCells[j]))
                    {
                        Propagator.MarkTriangleAsObserved(observedCells[j]);
                    }
                }
            }
        }

        /// <summary>
        /// Performs a Ray March along a given Ray, returns an array containing the indexes of observed cells in the NavMesh
        /// </summary>
        /// <param name="ray">The ray to march across</param>
        /// <param name="marchIntervalDistance">The interval between each march point</param>
        /// <param name="numberOfMarches">The number of marches to perform</param>
        /// <param name="maxDistance">The max distance to cut off at prematurely</param>
        /// <remarks>Max distance of the Ray is marchIntervalDistance * numberOfMarches. Stops prematurely once a point evaluates to not on the NavMesh</remarks>
        /// <returns>An array containing the indexes of the observed triangles in the NavMesh</returns>
        protected int[] RayMarch (Ray ray, float marchIntervalDistance, int numberOfMarches, float maxDistance)
        {
            float marchDistance = marchIntervalDistance;
            ProbNavMeshTriangulation navMesh = Propagator.GetProbabilityNavMeshTriangulation();

            List<int> observedCells = new List<int>();
            
            //Evaluate each point on the Ray March, stop if we go off the mesh or exceed our max distance
            for (int i = 1; i < numberOfMarches && marchDistance < maxDistance; i++, marchDistance += marchIntervalDistance)
            {
                Vector3 marchedPoint = ray.origin + (ray.direction * marchDistance);
                int? triangleindex = navMesh.EvaluatePoint(marchedPoint);

                if (triangleindex.HasValue)
                {
                    break;
                }
                else
                {
                    observedCells.Add(triangleindex.Value);
                }
            }

            return observedCells.ToArray();
        }

        /// <summary>
        /// Performs a raycast to the target to see if it is visible
        /// </summary>
        /// <param name="targetEntity">The entity to look at</param>
        /// <remarks>Uses the transform.position of the targetEntity so may have 
        /// some edge cases where a large model returns a false negative</remarks>
        /// <returns>True if there is an unobstructed line of sight to the enemy, False if there isn't</returns>
        public bool CanSeeTarget(GameObject targetEntity)
        {
            Vector3 dirToEntity = targetEntity.transform.position - transform.position;
            dirToEntity.Normalize();
            if (Physics.Raycast(transform.position, dirToEntity, out RaycastHit hitInfo, MaxLOSDistance, VisibilityLayerMask, QueryTriggerInteraction.Ignore))
            {
                if (hitInfo.collider.gameObject == targetEntity)
                {
                    return true;
                }
            }
            return false;
        }
    }
}

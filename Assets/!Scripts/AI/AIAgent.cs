using ProbabilityNavMesh.Commands;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace ProbabilityNavMesh.AI
{
    [RequireComponent(typeof(AICellObserver))]
    public class AIAgent : MonoBehaviour
    {
        [Header("Movement")]
        public float AgentSpeedIncrement = 0.25f;
        public float AgentMaxSpeed = 1.5f;
        public float MaxRotationAngleInDegrees = 5.0f;
        public float MinWaypointDistance = 0.1f;

        [Header("Propagator")]
        public VisibilityProbabilityPropagator Propagator;

        [Header("Target Entity")]
        public GameObject TargetEntity;

        private NavMeshPath currentPath = null;
        private int currentCornerIndex = 0;
        private AICommandFactory commandFactory = new AICommandFactory();
        private AICellObserver observer;
        private bool canSeeTarget = false;
        private bool prevCanSeeTarget = false;

        /// <summary>
        /// Called at the start, gets a reference to the cell observer
        /// </summary>
        private void Start()
        {
            observer = GetComponent<AICellObserver>();
        }

        /// <summary>
        /// Called once per frame, evaluates the current path and executes a specified command
        /// </summary>
        private void Update()
        {
            //Check if we can see the target
            prevCanSeeTarget = canSeeTarget;
            canSeeTarget = observer.CanSeeTarget(TargetEntity);

            //If we can see the target, if our current path is null or has been completed, or if the visibility state has changed
            if (canSeeTarget || (currentPath == null || currentCornerIndex > (currentPath.corners.Length - 1)) || canSeeTarget != prevCanSeeTarget)
            {
                // Calculate the path again
                currentPath = CalculatePath();
                currentCornerIndex = 0;
            }

            if (currentPath != null)
            {
                //Are we at our current goal node?
                float distToCorner = Vector3.Distance(currentPath.corners[currentCornerIndex], transform.position);
                if (distToCorner < MinWaypointDistance)
                {
                    //Increment to the next goal and return
                    currentCornerIndex++;
                    return;
                }

                //Begin moving towards the path
                ICommand commandToExecute = commandFactory.GetCommand(this, currentPath.corners[currentCornerIndex]);
                commandToExecute.ExecuteCommand(gameObject);
            }
        }

        /// <summary>
        /// Calculates the goal node of the agent and queries the NavMesh for a path to it
        /// </summary>
        /// <returns>YThe created Path</returns>
        private NavMeshPath CalculatePath()
        {
            Vector3 goalPosition;
            //If the observer can see the target then it is our goal, otherwise we use the highest probability
            if (canSeeTarget)
            {
                goalPosition = TargetEntity.transform.position;
            }
            else
            {
                //Get the highest probability triangle
                int targetPositionIndex = Propagator.GetHighestProbabilityIndex();
                ProbNavMeshTriangulation probNavMeshTriangulation = Propagator.GetProbabilityNavMeshTriangulation();
                if (targetPositionIndex < 0)
                {
                    Debug.LogError("ERROR: No Triangle Index Provided!");
                    return null;
                }

                //Get the vertices of the triangle
                int i1 = probNavMeshTriangulation.NavMeshTriangulationData.indices[targetPositionIndex];
                int i2 = probNavMeshTriangulation.NavMeshTriangulationData.indices[targetPositionIndex + 1];
                int i3 = probNavMeshTriangulation.NavMeshTriangulationData.indices[targetPositionIndex + 2];
                Vector3 v1 = probNavMeshTriangulation.NavMeshTriangulationData.vertices[i1];
                Vector3 v2 = probNavMeshTriangulation.NavMeshTriangulationData.vertices[i2];
                Vector3 v3 = probNavMeshTriangulation.NavMeshTriangulationData.vertices[i3];

                //Evaluate the mid point of the triangle
                //TODO: use actual equation lol
                Vector3 triangleMid = (v1 + v2 + v3) / 3f;

                goalPosition = triangleMid;
            }

            NavMeshPath path = new NavMeshPath();
            bool result = NavMesh.CalculatePath(transform.position, goalPosition, NavMesh.AllAreas, path);

            if (result)
            {
                return path;
            }
            else
            {
                Debug.LogError("ERROR: No Path Found!");
            }
            return null;
        }
    }
}

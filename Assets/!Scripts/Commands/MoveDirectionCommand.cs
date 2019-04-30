using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// A command implementation that moves in a set direction
    /// </summary>
    public class MoveDirectionCommand : ICommand
    {
        private Vector3 direction;
        private float speed;
        private float maxSpeed;
        private Rigidbody rigidbodyCache;

        /// <summary>
        /// Constructor that takes in the desired direction, speed, and max speed
        /// </summary>
        public MoveDirectionCommand(Vector3 newDirection, float newSpeed, float newMaxSpeed)
        {
            direction = newDirection;
            speed = newSpeed;
            maxSpeed = newMaxSpeed;
        }

        /// <summary>
        /// Moves the entity in a predefined direction, clamping at a predefined max. Uses the Rigidbody velocity system.
        /// </summary>
        public void ExecuteCommand(GameObject entityToExecuteOn)
        {
            if (!rigidbodyCache)
            {
                rigidbodyCache = entityToExecuteOn.GetComponentInChildren<Rigidbody>();
            }
            if (rigidbodyCache)
            {
                rigidbodyCache.velocity += direction * speed;
                rigidbodyCache.velocity = Vector3.ClampMagnitude(rigidbodyCache.velocity, maxSpeed);
            }
        }
    }
}

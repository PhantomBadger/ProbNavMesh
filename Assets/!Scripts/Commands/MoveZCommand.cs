using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// A command implementation that moves the entity along the Z axis via Rigidbody velocities
    /// </summary>
    public class MoveZCommand : ICommand
    {
        private float speed;
        private float maxSpeed;
        private Rigidbody rigidbodyCache;

        /// <summary>
        /// Constructor for the MoveZCommand Command, takes in a speed and max speed
        /// </summary>
        /// <param name="newSpeed">The speed of the movement</param>
        /// <param name="newMaxSpeed">The max speed to clamp movement at</param>
        public MoveZCommand(float newSpeed, float newMaxSpeed)
        {
            speed = newSpeed;
            maxSpeed = newMaxSpeed;
        }

        /// <summary>
        /// Moves the entity along the Vector3.forward by speed, clamps the magnitude at max
        /// </summary>
        public void ExecuteCommand(GameObject entityToExecuteOn)
        {
            if (!rigidbodyCache)
            {
                rigidbodyCache = entityToExecuteOn.GetComponentInChildren<Rigidbody>();
            }
            if (rigidbodyCache)
            {
                rigidbodyCache.velocity += Vector3.forward * speed;
                rigidbodyCache.velocity = Vector3.ClampMagnitude(rigidbodyCache.velocity, maxSpeed);
            }
        }
    }
}

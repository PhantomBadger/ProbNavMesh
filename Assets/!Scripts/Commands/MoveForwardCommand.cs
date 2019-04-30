using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// Implementation of the Command interface to move the entity along it's forward axis
    /// </summary>
    public class MoveForwardCommand : ICommand
    {
        private float speed;
        private float maxSpeed;
        private Rigidbody rigidbodyCache;

        /// <summary>
        /// Constructor for the MoveForward Command, takes in a speed and max speed
        /// </summary>
        /// <param name="newSpeed">The speed of the movement</param>
        /// <param name="newMaxSpeed">The max speed to clamp movement at</param>
        public MoveForwardCommand(float newSpeed, float newMaxSpeed)
        {
            speed = newSpeed;
            maxSpeed = newMaxSpeed;
        }

        /// <summary>
        /// Moves the entity along the forward vector by speed, clamps the magnitude at max
        /// </summary>
        public void ExecuteCommand(GameObject entityToExecuteOn)
        {
            if (!rigidbodyCache)
            {
                rigidbodyCache = entityToExecuteOn.GetComponentInChildren<Rigidbody>();
            }
            if (rigidbodyCache)
            {
                rigidbodyCache.velocity += entityToExecuteOn.transform.forward * speed;
                rigidbodyCache.velocity = Vector3.ClampMagnitude(rigidbodyCache.velocity, maxSpeed);
            }
        }
    }
}

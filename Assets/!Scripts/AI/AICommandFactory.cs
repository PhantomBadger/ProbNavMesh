using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ProbabilityNavMesh.Commands;

namespace ProbabilityNavMesh.AI
{
    public class AICommandFactory
    {
        /// <summary>
        /// Returns an implementation of ICommand for the AI Agent to execute in order to arrive at it's goal position
        /// </summary>
        /// <param name="entityToGetCommandFor">The entity to evaluate to find the correct command</param>
        /// <param name="goalPosition">The position to work towards</param>
        /// <returns>A suitable implementation of ICommand</returns>
        public ICommand GetCommand(AIAgent entityToGetCommandFor, Vector3 goalPosition)
        {
            Vector3 goalForward = Vector3.ProjectOnPlane(goalPosition - entityToGetCommandFor.transform.position, Vector3.up);

            //Create the movement command
            float speedIncrement = entityToGetCommandFor.AgentSpeedIncrement;
            float maxSpeed = entityToGetCommandFor.AgentMaxSpeed;
            MoveDirectionCommand command = new MoveDirectionCommand(goalForward, speedIncrement, maxSpeed);

            Debug.Log("MOVE V: " + goalForward.ToString("n4") + " S: " + speedIncrement);

            //This method can now be expanded should any additional movement types be needed
            return command;
        }
    }
}

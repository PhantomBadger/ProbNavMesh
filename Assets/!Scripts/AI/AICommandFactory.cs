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
            Vector3 entityForward = entityToGetCommandFor.transform.forward;
            Vector3 goalForward = goalPosition = entityToGetCommandFor.transform.position;
            float angleToGoal = Vector3.Angle(entityForward, goalForward);

            //Create the movement and rotation command
            float rotateAngle = Mathf.Min(angleToGoal, entityToGetCommandFor.MaxRotationAngleInDegrees);
            float speedIncrement = entityToGetCommandFor.AgentSpeedIncrement;
            float maxSpeed = entityToGetCommandFor.AgentMaxSpeed;
            MoveAndRotateCommand command = new MoveAndRotateCommand(rotateAngle, goalForward, speedIncrement, maxSpeed);

            //This method cna now be expanded should any additional movement types be needed
            return command;
        }
    }
}

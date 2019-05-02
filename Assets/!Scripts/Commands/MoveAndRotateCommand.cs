using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// A command implementation that combined the Rotate and MoveDirection commands
    /// </summary>
    public class MoveAndRotateCommand : ICommand
    {
        private ICommand rotateCommand;
        private ICommand moveCommand;

        /// <summary>
        /// Constructor that takes in the values needed for a rotate and move command
        /// </summary>
        public MoveAndRotateCommand(float newAngle, Vector3 newDirection, float newSpeed, float newMaxSpeed)
        {
            rotateCommand = new RotateCommand(newAngle);
            moveCommand = new MoveDirectionCommand(newDirection, newSpeed, newMaxSpeed);
        }

        /// <summary>
        /// Rotates the entity and moves it in a predefined direction
        /// </summary>
        public void ExecuteCommand(GameObject entityToExecuteOn)
        {
            rotateCommand.ExecuteCommand(entityToExecuteOn);
            moveCommand.ExecuteCommand(entityToExecuteOn);
        }
    }
}
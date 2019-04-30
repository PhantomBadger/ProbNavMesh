using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// A command implementation that represents a rotation by a given number of degrees
    /// </summary>
    public class RotateCommand : ICommand
    {
        private float angle;

        /// <summary>
        /// Constructor for the command, taking in the angle you want to rotate around the entity's up vector by
        /// </summary>
        public RotateCommand(float newAngle)
        {
            angle = newAngle;
        }

        /// <summary>
        /// Rotates the entity around it's up vector by a predefined angle
        /// </summary>
        public void ExecuteCommand(GameObject entityToExecuteOn)
        {
            entityToExecuteOn.transform.forward = Quaternion.AngleAxis(angle, entityToExecuteOn.transform.up) * entityToExecuteOn.transform.forward;
        }
    }
}

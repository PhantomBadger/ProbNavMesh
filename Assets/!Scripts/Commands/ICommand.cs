using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ProbabilityNavMesh.Commands
{
    /// <summary>
    /// Command interface to facilitate multiple commands being used by an entity
    /// </summary>
    public interface ICommand
    {
        void ExecuteCommand(GameObject entityToExecuteOn);
    }
}

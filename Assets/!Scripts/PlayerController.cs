using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using ProbabilityNavMesh.Commands;

namespace ProbabilityNavMesh
{
    public class PlayerController : MonoBehaviour
    {
        public float SpeedIncrement = 0.25f;
        public float MaxSpeed = 1.5f;

        private Dictionary<KeyCode, ICommand> keyboardMap;

        /// <summary>
        /// Called at the start, Calls the initialise methods
        /// </summary>
        private void Start()
        {
            InitialiseKeyboardBindings();
        }

        /// <summary>
        /// Initialises the keyboard bindings
        /// </summary>
        private void InitialiseKeyboardBindings()
        {
            keyboardMap = new Dictionary<KeyCode, ICommand>()
                {
                    {KeyCode.W, new MoveZCommand(SpeedIncrement, MaxSpeed)},
                    {KeyCode.A, new MoveXCommand(-SpeedIncrement, MaxSpeed)},
                    {KeyCode.S, new MoveZCommand(-SpeedIncrement, MaxSpeed)},
                    {KeyCode.D, new MoveXCommand(SpeedIncrement, MaxSpeed)}
                };
        }

        /// <summary>
        /// Called once a frame, initiates the processing of inputs
        /// </summary>
        private void Update()
        {
            ProcessInput();
        }

        /// <summary>
        /// Process the user input using the keyboardMap dictionary
        /// </summary>
        private void ProcessInput()
        {
            if (keyboardMap == null)
            {
                return;
            }

            //Iterate over each binding and execute the command if needed
            foreach(KeyValuePair<KeyCode, ICommand> binding in keyboardMap)
            {
                if (Input.GetKey(binding.Key))
                {
                    if (binding.Value != null)
                    {
                        binding.Value.ExecuteCommand(this.gameObject);
                    }
                }
            }
        }
    }
}

using UnityEngine;
using System;
using AWSIM.Scripts.UI;
using AWSIM.TrafficSimulation;

namespace AWSIM
{
    /// <summary>
    /// Script for Scene. By Scirpt-Execution-Order, This class is called earlier than other MonoBehaviour.
    /// Enables configuration at scene startup.
    /// </summary>
    public class AutowareSimulation : MonoBehaviour
    {
        [SerializeField] TrafficManager trafficManager;
        [SerializeField] Transform egoTransform;
        [SerializeField] TimeSourceSelector timeSourceSelector;
        [SerializeField] private int targetFramerate;

        [Header("Player Config")]
        [SerializeField] string commandLineConfigParam = "--json_path";

        [Header("Editor Debug")]
        [SerializeField] bool useJsonConfig;

        [Tooltip("Specify this Path if you want to debug Json loading at Unity Editor runtime.")]
        [SerializeField] string jsonPath;

        [Serializable]
        public class EgoConfiguration
        {
            public Vector3 Position;
            public Vector3 EulerAngles;
        }

        [Serializable]
        public class Configuration
        {
            public float TimeScale;                 // Reflected in Time.timeScale
            public string TimeSource;               // Reflected in TimeSourceSelector
            public int RandomTrafficSeed;           // Reflected in TrafficManager.seed
            public int MaxVehicleCount;             // Reflected in TrafficManager.maxVehicleCount
            public EgoConfiguration Ego = new EgoConfiguration();
        }

        private EgoVehiclePositionManager egoVehiclePositionManager;
        private TrafficControlManager trafficControlManager;

        void Awake()
        {
            // check if time source selector is present
            if (timeSourceSelector == null)
            {
                Debug.LogWarning("TimeSource: There is no TimeSourceSelector object assigned in the inspector. The default time source will be used.");
            }

#if !UNITY_EDITOR
            // initialize
            useJsonConfig = false;
            jsonPath = "";

            // For player, if path is not specified as a command line argument, json is not read.
            useJsonConfig = CommandLineUtility.GetCommandLineArg(out jsonPath, commandLineConfigParam);
#endif

            if (useJsonConfig)
            {
                var config = CommandLineUtility.LoadJsonFromPath<Configuration>(jsonPath);
                Time.timeScale = config.TimeScale;
                trafficManager.seed = config.RandomTrafficSeed;
                trafficManager.maxVehicleCount = config.MaxVehicleCount;

                var position = config.Ego.Position - Environment.Instance.MgrsOffsetPosition;
                egoTransform.position = ROS2Utility.RosToUnityPosition(position);

                var rotation = Quaternion.Euler(config.Ego.EulerAngles);
                egoTransform.rotation = ROS2Utility.RosToUnityRotation(rotation);

                // set time source
                timeSourceSelector?.SetType(config.TimeSource);
            }

            // Add EgoVehiclePositionManager
            if (!egoTransform)
            {
                Debug.LogWarning("Missing EgoTransform reference.");
            }
            else
            {
                egoVehiclePositionManager = gameObject.AddComponent<EgoVehiclePositionManager>();
                egoVehiclePositionManager.EgoTransform = egoTransform;
                egoVehiclePositionManager.enabled = true;
            }

            // Add TrafficControlManager
            if (!trafficManager)
            {
                Debug.LogWarning("Missing TrafficManager reference. ");
            }
            else
            {
                trafficControlManager = gameObject.AddComponent<TrafficControlManager>();
                trafficControlManager.TrafficManager = trafficManager;
                trafficControlManager.enabled = true;
            }
        }
        private void Start()
        {
            Application.targetFrameRate = targetFramerate;
        }
    }
}

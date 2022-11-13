using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class Publisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "world/base_link/base_link_inertia/shoulder_link", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link", "/wrist_3_link" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/ur5e_unity_joint_state";
    [SerializeField]
    public GameObject m_ur5e;

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<JointStateMsg>(m_TopicName); 
        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_ur5e.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    public void Publish()
    {
    	var joint_state = new JointStateMsg();

        joint_state.position = new Double[6];
        
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            joint_state.position[i]= m_JointArticulationBodies[i].GetPosition();
        }

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, joint_state);
    }
    
    public void Update()
    {
    	Publish();
    }
}

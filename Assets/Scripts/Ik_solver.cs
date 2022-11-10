using System;
using System.Collections;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class Ik_solver : MonoBehaviour
{

    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    public static readonly string[] LinkNames =
    { "/shoulder_links", "/upper_arm_link", "/forearm_link", "/wrist_1_link", "/wrist_2_link","wrist_3_link" };
    // Variables required for ROS communication
    [SerializeField]
    GameObject m_UR5e;
    public GameObject UR5e { get => m_UR5e; set => m_UR5e = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    //readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    //readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    float computeIK(GameObject target){
        return 30;
    }
    void Start()
    {
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_UR5e.transform.Find(linkName).GetComponent<ArticulationBody>();
            Debug.Log(m_UR5e.transform.Find(linkName));
        }

    }

    void Update()
    {
        StartCoroutine(waiter());
        // Set the joint values for every joint
    }
 IEnumerator waiter(){
        for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
        {
            var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
            joint1XDrive.target = computeIK(Target);
            Debug.Log(joint +" set to :" + joint1XDrive.target);
            m_JointArticulationBodies[joint].xDrive = joint1XDrive;
        }
        yield return   new WaitForSeconds(2);
       
    } 
        


}

using RosMessageTypes.MotionPlan;
using RosMessageTypes.Std;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.UrdfImporter.Control;

public class GripperController : MonoBehaviour
{
    private bool closed;

    private string TopicName = "/gripper_cmd";

    private ArticulationBody[] JointList;

    private ArticulationDrive currentDrive;

    private ArticulationBody[] articulationChain;

    private const float GRIPPER_JOINT_ANGLE_CLOSE = -17.0f;

    private const float GRIPPER_JOINT_ANGLE_OPEN = 0f;

    public bool is_the_real_robot = false;

    ROSConnection m_Ros;

    // Start is called before the first frame update
    void Start()
    {
        closed = false;
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<Gripper_cmdMsg> (TopicName);
        JointList = new ArticulationBody[2];
        foreach (ArticulationBody joint in articulationChain)
        {
            switch (joint.name)
            {
                case "soft_robotics_right_finger_link1":
                    JointList[0] = joint;
                    break;
                case "soft_robotics_left_finger_link1":
                    JointList[1] = joint;
                    break;
            }
        }
    }

    // Update is called once per frame
    void Update()
    {
        OVRInput.Update();

        //pressed A
        //if (Input.GetKeyDown("space")){
        if (button_close_pressed())
        {
            if (closed == false)
            {
                closed = true;
                close_gripper();
                return;
            }
        }
        if (button_open_pressed())
        {
            // if (Input.GetKeyDown("up")){
            if (closed == true)
            {
                closed = false;
                open_gripper();
                return;
            }
        }
    }

    void open_gripper()
    {
        var closure = new Gripper_cmdMsg();
        closure.close = false;

        //So i am publishing the msg just for the simulated robot
        if (!is_the_real_robot)
        {
            m_Ros.Publish (TopicName, closure);
        }

        ArticulationDrive currentDrive = JointList[0].xDrive;
        currentDrive.target = GRIPPER_JOINT_ANGLE_OPEN;
        JointList[0].xDrive = currentDrive;
        JointList[1].xDrive = currentDrive;
    }

    void close_gripper()
    {
        var closure = new Gripper_cmdMsg();
        closure.close = true;

        //So i am publishing the msg just for the simulated robot
        if (!is_the_real_robot)
        {
            m_Ros.Publish (TopicName, closure);
        }
        ArticulationDrive currentDrive = JointList[0].xDrive;
        currentDrive.target = GRIPPER_JOINT_ANGLE_CLOSE;
        JointList[0].xDrive = currentDrive;
        JointList[1].xDrive = currentDrive;
    }

    bool button_open_pressed()
    {
        if (Application.isEditor)
        {
            return Input.GetKeyDown("space");
        }
        else
        {
            return OVRInput.Get(OVRInput.Button.Two);
        }
    }

    bool button_close_pressed()
    {
        if (Application.isEditor)
        {
            return Input.GetKeyDown("space");
        }
        else
        {
            return OVRInput.Get(OVRInput.Button.One);
        }
    }
}

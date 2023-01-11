using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.MotionPlan;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

using Unity.Robotics.UrdfImporter.Control;

public class Real_Robot_Feedback_Controller : MonoBehaviour
{
    public enum RotationDirection
    {
        None = 0,
        Positive = 1,
        Negative = -1
    }

    public enum ControlType
    {
        PositionControl
    }

    [Header("Robot Control References")]

    private ArticulationBody[] articulationChain;

    private ArticulationBody[] JointList;


    private int previousIndex;

    public GameObject ur5e;

    //[InspectorReadOnly(hideInEditMode: true)]
    //public string selectedJoint;

    [HideInInspector]
    public int selectedIndex;

    [Header("Robot properties")]
    public ControlType control = ControlType.PositionControl;

    public float stiffness;

    public float damping;

    public float forceLimit;

    public float speed = 5f; // Units: degree/s

    public float torque = 100f; // Units: Nm or N

    public float acceleration = 5f; // Units: m/s^2 / degree/s^2

    public bool is_topic_published = false;

    private int frame_num;

    [Tooltip("Color to highlight the currently selected Join")]
    public Color highLightColor = new Color(1, 0, 0, 1);

    //The old controller
    private Controller controller;

    private void OnEnable()
    {
        this.gameObject.AddComponent(typeof (Controller));
        controller = GetComponent<Controller>();
        SetControllerValues();
    }

    void Start()
    {   
        ROSConnection.GetOrCreateInstance().Subscribe<JointAnglesMsg>("/real_robot_joint_state", callback);
        frame_num=0;
        previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        JointList = new ArticulationBody[6];
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            switch (joint.name)
            {
                case "shoulder_link":
                    JointList[0] = joint; 
                    break;
                case "upper_arm_link": 
                    JointList[1] = joint; 
                    break;  
                case "forearm_link": 
                    JointList[2] = joint; 
                    break;
                case "wrist_1_link":
                    JointList[3] = joint; 
                    break;
                case "wrist_2_link":
                    JointList[4] = joint; 
                    break;
                case "wrist_3_link":
                    JointList[5] = joint; 
                    break;
                default:
                    break;
            }
        }
    }
    void Update()
    {   
        //check if we are receveing messages in the last 200 frames
        if(frame_num>300 && is_topic_published==true){
            is_topic_published = false;
        }
        frame_num+=1;
    }

    void UpdateControlType(JointControl joint)
    {
        joint.controltype =
            Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
        if (control == ControlType.PositionControl)
        {
            ArticulationDrive drive = joint.joint.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            joint.joint.xDrive = drive;
        }
    }

    void SetControllerValues()
    {
        controller.stiffness = stiffness;
        controller.damping = damping;
        controller.forceLimit = forceLimit;
        controller.speed = speed;
        controller.torque = torque;
        controller.acceleration = acceleration;
    }

    double Rad2Deg(double radx)
    {
        return radx * (180 / 3.141592653589793238463);
    }

    //set angles of real robot    
    void callback(JointAnglesMsg angles){
        //restart counter if we receive a message
        frame_num=0;
        is_topic_published=true;
        int i=0;
        foreach (ArticulationBody joint in JointList){
        ArticulationDrive currentDrive = joint.xDrive;
        currentDrive.target = (float)Rad2Deg(angles.joint_angles[i]);
        joint.xDrive = currentDrive;
        i++;
        }
    }

}

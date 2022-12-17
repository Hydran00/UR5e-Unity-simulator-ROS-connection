using EigenCore.Core.Dense;
using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;

public class RobotController : MonoBehaviour
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
    public GameObject Target;

    private UR5e_IKS IK;

    private UR5e_InvDiffKin IDK;

    private ArticulationBody[] articulationChain;

    private ArticulationBody[] JointList;

    private UrdfJointRevolute[] Real_Robot_Joints;

    private Color[] prevColor;

    private int previousIndex;

    public GameObject ur5e;

    //real ur5e is the current state(feedback) of the real robot
    public GameObject real_ur5e;

    public static readonly string[]
        LinkNames =
        {
            "world/base_link/base_link_inertia/shoulder_link",
            "/upper_arm_link",
            "/forearm_link",
            "/wrist_1_link",
            "/wrist_2_link",
            "/wrist_3_link"
        };

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

    private int current_frame;

    private int frame_num;

    private MatrixXD res;

    [Tooltip("Color to highlight the currently selected Join")]
    public Color highLightColor = new Color(1, 0, 0, 1);

    //The old controller
    private Controller controller;

    private void OnEnable()
    {
        this.gameObject.AddComponent(typeof (Controller));
        controller = GetComponent<Controller>();
        SetControllerValues();
        current_frame = 0;
        frame_num = 0;
        res = new MatrixXD(new double[10, 6]);
    }

    void Start()
    {
        previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        JointList = new ArticulationBody[6];
        Real_Robot_Joints = new UrdfJointRevolute[6];
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
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

        //fill real robot joint
        var linkName = string.Empty;
        for (var i = 0; i < 6; i++)
        {
            linkName += LinkNames[i];
            Real_Robot_Joints[i] =
                real_ur5e
                    .transform
                    .Find(linkName)
                    .GetComponent<UrdfJointRevolute>();
        }
        IDK = new UR5e_InvDiffKin(Target, ur5e);
    }

    private void FixedUpdate()
    {
        MoveRobot();
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

    void MoveRobot()
    {
        check_target_pos();

        //set controllable virtual robot in the same position of real robot one
        if (frame_num < 150)
        {
            init_angles();
            frame_num++;
            return;
        }
        else
        {
            //Compute inv. diff. kinematics 
            res = IDK.ComputeInvDiffKin();

            int i = 0;
            //apply computed joints angles
            foreach (ArticulationBody joint in JointList)
            {
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.target = (float) Rad2Deg(res.Get(0, i));
                joint.xDrive = currentDrive;
                i++;
            }
        }
    }
    
    
    
    
    
    
    //limit target's position space
    void check_target_pos()
    {
        //limit x position
        if (Target.transform.position.x > 0.51)
        {
            Target.transform.position =
                new Vector3(0.51f,
                    Target.transform.position.y,
                    Target.transform.position.z);
        }
        if (Target.transform.position.x < -0.51)
        {
            Target.transform.position =
                new Vector3(-0.51f,
                    Target.transform.position.y,
                    Target.transform.position.z);
        }

        //limit y position
        if (Target.transform.position.y > 1.3)
        {
            Target.transform.position =
                new Vector3(Target.transform.position.x,
                    1.3f,
                    Target.transform.position.z);
        }
        if (Target.transform.position.y < 0.85)
        {
            Target.transform.position =
                new Vector3(Target.transform.position.x,
                    0.85f,
                    Target.transform.position.z);
        }

        //limit z position
        if (Target.transform.position.z > 0.05)
        {
            Target.transform.position =
                new Vector3(Target.transform.position.x,
                    Target.transform.position.y,
                    0.05f);
        }
        if (Target.transform.position.z < -0.46)
        {
            Target.transform.position =
                new Vector3(Target.transform.position.x,
                    Target.transform.position.y,
                    -0.46f);
        }
    }
    
    
    
    
    
    void init_angles(){
            //check if we are connected to real robot (that means at least one joint is not zero)
            bool connected_to_real_robot = false;
            for (int j = 0; j < 6; j++)
            {
                if (Mathf.Abs(Real_Robot_Joints[j].GetPosition()) > 0.001)
                {
                    Debug.Log("connected because "+j+" is "+Real_Robot_Joints[j].GetPosition());
                    connected_to_real_robot = true;
                    break;
                }
            }

            //Assign hardcoded values to joint when we don't have real robot joint published yet
            if (!connected_to_real_robot)
            {
                set_harcoded_angles();
            }
            else
            //if we're connected to robot then apply real robot angles to simulated robot to start simulation safely
            {
                int j = 0;
                foreach (ArticulationBody joint in JointList)
                {
                    ArticulationDrive currentDrive = joint.xDrive;
                    currentDrive.target =
                        (float) Rad2Deg(Real_Robot_Joints[j].GetPosition());
                    joint.xDrive = currentDrive;
                    j++;
                }

                //move target in end effector position for safety reasons
                VectorXD qk = VectorXD.Zeros(6);
                for (int k = 0; k < 6; k++)
                {
                    qk.Set(k, Real_Robot_Joints[k].GetPosition());
                }

                //need to compute fk to get ee's current position
                MatrixXD fk_res = IDK.FK.forward(qk);
                Vector3 pos =
                    new Vector3((float) - fk_res.Get(0, 3),
                        1.79f - (float) fk_res.Get(2, 3),
                        (float) fk_res.Get(1, 3));
                Debug.Log (pos);
                Target.transform.position = pos;
            }
    }
    void set_harcoded_angles()
    {
        foreach (ArticulationBody joint in JointList)
        {
            ArticulationDrive currentDrive = new ArticulationDrive();
            switch (joint.name)
            {
                case "shoulder_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = -91f;
                    joint.xDrive = currentDrive;
                    break;
                case "upper_arm_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = -60f;
                    joint.xDrive = currentDrive;
                    break;
                case "forearm_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = -131f;
                    joint.xDrive = currentDrive;
                    break;
                case "wrist_1_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = -84f;
                    joint.xDrive = currentDrive;
                    break;
                case "wrist_2_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = -90f;
                    joint.xDrive = currentDrive;
                    break;
                case "wrist_3_link":
                    currentDrive = joint.xDrive;
                    currentDrive.target = 0f;
                    joint.xDrive = currentDrive;
                    break;
                default:
                    break;
            }
        }
    }
}

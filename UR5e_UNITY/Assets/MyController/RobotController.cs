using EigenCore.Core.Dense;
using UnityEngine;
using Unity.Robotics;
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

    private Color[] prevColor;

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

    private int current_frame;

    private int frame_num;

    private MatrixXD res;

    [Tooltip("Color to highlight the currently selected Join")]
    public Color highLightColor = new Color(1, 0, 0, 1);

    private double[] q_sols;

    private double[] T;

    //The old controller
    private Controller controller;

    private void OnEnable()
    {
        this.gameObject.AddComponent(typeof (Controller));
        controller = GetComponent<Controller>();
        SetControllerValues();
        float[] A = new float[] { 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 1f };
        T = new double[16];
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
        q_sols = new double[6];
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
            
        //set robot in a safe position to avoid singularity
        if (frame_num == 1)
        {
            foreach (ArticulationBody joint in JointList)
            {
                ArticulationDrive currentDrive = new ArticulationDrive();
                switch (joint.name)
                {
                    case "shoulder_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = 69f;
                        joint.xDrive = currentDrive;
                        break;
                    case "upper_arm_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = -102f;
                        joint.xDrive = currentDrive;
                        break;
                    case "forearm_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = 109f;
                        joint.xDrive = currentDrive;
                        break;
                    case "wrist_1_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = -102f;
                        joint.xDrive = currentDrive;
                        break;
                    case "wrist_2_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = 89f;
                        joint.xDrive = currentDrive;
                        break;
                    case "wrist_3_link":
                        currentDrive = joint.xDrive;
                        currentDrive.target = 9f;
                        joint.xDrive = currentDrive;
                        break;
                    default:
                        break;
                }
            }
            frame_num++;
            return;
        }
        if (frame_num < 60){
            frame_num++;
            return;
        }  
        if (current_frame == 0)
        {
            res = IDK.ComputeInvDiffKin();
        }
        int i=0;
        foreach (ArticulationBody joint in JointList){
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.target = (float)Rad2Deg(res.Get(current_frame, i));
            joint.xDrive = currentDrive;
            i++;
        }
        current_frame = (current_frame + 1)% IDK.num_steps_per_frame;
        
    }
    void check_target_pos(){
        
        //limit x position
        if(Target.transform.position.x > 0.51){
            Target.transform.position = new Vector3(0.51f, Target.transform.position.y, Target.transform.position.z);
        
        }
        if (Target.transform.position.x <-0.51){
            Target.transform.position = new Vector3(-0.51f, Target.transform.position.y, Target.transform.position.z);
        }
        //limit y position
        if(Target.transform.position.y > 1.3){
            Target.transform.position = new Vector3(Target.transform.position.x, 1.3f, Target.transform.position.z);
        
        }
        if (Target.transform.position.y <0.85){
            Target.transform.position = new Vector3(Target.transform.position.x, 0.85f, Target.transform.position.z);
        }
        //limit z position
        if(Target.transform.position.z > 0.05){
            Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y,0.05f);
        
        }
        if (Target.transform.position.z <-0.46){
            Target.transform.position = new Vector3(Target.transform.position.x, Target.transform.position.y,-0.46f);
        }
    }
    /*
    void prepare_matrix()
    {
        for (int i = 0; i < 16; i++)
        {
            T[i] = 0;
        }
        double H_TABLE = 1.79;
        T[3] = Target.transform.position.x;
        T[7] = -Target.transform.position.z;
        T[11] = H_TABLE - Target.transform.position.y;
        Matrix4x4 homogeneous_trans_matrix =
            Matrix4x4
                .TRS(Vector3.zero,
                ConvertToUnity(Target.transform.rotation),
                Vector3.one);

        T[0] = homogeneous_trans_matrix[0, 0];
        T[1] = homogeneous_trans_matrix[0, 1];
        T[2] = homogeneous_trans_matrix[0, 2];
        T[4] = homogeneous_trans_matrix[1, 0];
        T[5] = homogeneous_trans_matrix[1, 1];
        T[6] = homogeneous_trans_matrix[1, 2];
        T[8] = homogeneous_trans_matrix[2, 0];
        T[9] = homogeneous_trans_matrix[2, 1];
        T[10] = homogeneous_trans_matrix[2, 2];

        T[15] = 1;
    }

    //convert RH to LH coordinates for the homogeneous transf. matrix
    Quaternion ConvertToUnity(Quaternion input)
    {
        return new Quaternion(-input.x, input.z, input.y, input.w);
    }*/

}

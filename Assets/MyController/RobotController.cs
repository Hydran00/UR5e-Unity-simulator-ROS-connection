using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;

public class RobotController : MonoBehaviour
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };

    [Header("Robot Control References")]

    public GameObject Target;
    private UR5e_IKS IK;
    private ArticulationBody[] articulationChain;
    private Color[] prevColor;
    private int previousIndex;
    [InspectorReadOnly(hideInEditMode: true)]
    public string selectedJoint;
    [HideInInspector]
    public int selectedIndex;
    [Header("Robot properties")]
    public ControlType control = ControlType.PositionControl;
    public float stiffness;
    public float damping;
    public float forceLimit;
    public float speed = 5f; // Units: degree/s
    public float torque = 100f; // Units: Nm or N
    public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    [Tooltip("Color to highlight the currently selected Join")]
    public Color highLightColor = new Color(1, 0, 0, 1);
    private double[] q_sols;
    private double[] T;
    //The old controller
    private Controller controller;
    private void OnEnable()
    {
        this.gameObject.AddComponent(typeof(Controller));
        controller = GetComponent<Controller>();
        SetControllerValues();
        IK = new UR5e_IKS();
        T =new double[16];
    }

    void Start()
    {
        previousIndex = selectedIndex = 1;
        this.gameObject.AddComponent<FKRobot>();
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            joint.gameObject.AddComponent<JointControl>();
            joint.jointFriction = defDyanmicVal;
            joint.angularDamping = defDyanmicVal;
            ArticulationDrive currentDrive = joint.xDrive;
            currentDrive.forceLimit = forceLimit;
            joint.xDrive = currentDrive;
        }
        q_sols = new double[48];
    }

    private void FixedUpdate()
    {
        MoveRobot();
        
    }

    void UpdateControlType(JointControl joint)
    {
        joint.controltype = Unity.Robotics.UrdfImporter.Control.ControlType.PositionControl;
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
    double Rad2Deg(double radx){
        return radx * (180.0f/3.141592653589793238463f);
    }
    void MoveRobot()
    {
        //check if position or rotation is changed
        /*
        if (Target.transform.hasChanged)
        {
            Target.transform.hasChanged = false;
            return; 

        }*/
        //compute input matrix and storing it in T
        prepare_matrix();
        int sol_num = IK.inverse(T,q_sols,0.0);
        //set default IK solution to 0
        int i =0;
        Debug.Log(Rad2Deg(q_sols[i*6+0])+" "+ Rad2Deg(q_sols[i*6+1])+" "+ Rad2Deg(q_sols[i*6+2])+" "+ Rad2Deg(q_sols[i*6+3])+" "+ Rad2Deg(q_sols[i*6+4])+" "+ Rad2Deg(q_sols[i*6+5]));
        //Applying new joint target to xDrive of each joint
        foreach (ArticulationBody joint in articulationChain)
        {
            ArticulationDrive currentDrive = new ArticulationDrive();
            switch(joint.name){
                case "shoulder_link": 
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+0]); 
                Debug.Log("Assegno shoulder_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                case "upper_arm_link":
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+1]);
                Debug.Log("Assegno upper_arm_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                case "forearm_link":
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+2]);
                Debug.Log("Assegno forearm_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                case "wrist_1_link":
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+3]);
                Debug.Log("Assegno wrist_1_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                case "wrist_2_link":
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+4]);
                Debug.Log("Assegno wrist_2_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                case "wrist_3_link":
                currentDrive = joint.xDrive;
                currentDrive.target = (float)Rad2Deg(q_sols[i*6+5]);
                Debug.Log("Assegno wrist_3_link con : "+currentDrive.target);
                joint.xDrive = currentDrive;
                break;

                default: break;
            }
        }
    }
    void prepare_matrix() {
        for(int i=0;i<16;i++) {
            T[i]=0;
        }
        double H_TAVOLO = 0.63;
        T[3]= Target.transform.position.z;
        T[7]= -Target.transform.position.x; 
        T[11]= (Target.transform.position.y-H_TAVOLO);
               
        Matrix4x4 homogeneous_trans_matrix = Matrix4x4.TRS(Vector3.zero,
        ConvertToUnity(Target.transform.rotation), Vector3.one);
        Debug.Log(homogeneous_trans_matrix);

        T[0] = homogeneous_trans_matrix[0,0];
        T[1] = homogeneous_trans_matrix[0,1];
        T[2] = homogeneous_trans_matrix[0,2];
        T[4] = homogeneous_trans_matrix[1,0];
        T[5] = homogeneous_trans_matrix[1,1];
        T[6] = homogeneous_trans_matrix[1,2];
        T[8] = homogeneous_trans_matrix[2,0];
        T[9] = homogeneous_trans_matrix[2,1];
        T[10] = homogeneous_trans_matrix[2,2];

        T[15]=1;
        string output = "T__:" ;
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                output += " "+T[i*4+j];
            }
            output+="\n";
        }
        Debug.Log(output);
    }
    //convert RH to LH coordinates for the homogeneous transf. matrix
    Quaternion ConvertToUnity(Quaternion input) {
        return new Quaternion(-input.z,  input.x,-input.y,input.w);
    }

}
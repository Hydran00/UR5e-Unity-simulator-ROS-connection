using UnityEngine;
using Unity.Robotics;
using Unity.Robotics.UrdfImporter.Control;
//using UrdfControlRobot = Unity.Robotics.UrdfImporter.Control;

public class RobotController : MonoBehaviour
{
    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };
    //Fields for inputs
    [Header("Robot Control References")]
    //[SerializeField] private InputActionProperty moveJointInput;
    //[SerializeField] private InputActionProperty selectJointInput;
    //Robot properties
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
    public double[] T;

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
            //currentDrive.target = 30f;
            joint.xDrive = currentDrive;
        }
        q_sols = new double[48];
        int counter = 0;
        
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
        Transform target_pos = Target.transform;
        prepare_matrix();
        int sol_num = IK.inverse(T,q_sols,0.0);
        //for(int i=0;i<sol_num;i++) {
    int i =0,counter=0;
    Debug.Log(Rad2Deg(q_sols[i*6+0])+" "+ Rad2Deg(q_sols[i*6+1])+" "+ Rad2Deg(q_sols[i*6+2])+" "+ Rad2Deg(q_sols[i*6+3])+" "+ Rad2Deg(q_sols[i*6+4])+" "+ Rad2Deg(q_sols[i*6+5]));
        
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
            counter++;
        }

        
    }
    double[] prepare_matrix() {

        for(int i=0;i<16;i++) {
            T[i]=0;
        }
        double H_TAVOLO = 0.63;
        T[0]=1;T[5]=1;T[10]=1;T[15]=1;  
        //x pos
        T[3]= Target.transform.position.z;
        //y pos
        T[7]= -Target.transform.position.x;
        //z pos
        T[11]= (Target.transform.position.y-H_TAVOLO);
        Vector3 rotation = ToEulerAngles(Target.transform.rotation);
        T[0]= 1;//rotation.x;
        T[5]= 1;//rotation.y;
        T[10]= 1;//rotation.z;
        return T;
    }
    Vector3 ToEulerAngles(Quaternion q)
    {
        Vector3 angles = new();

        // roll / x
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.x = (float)Mathf.Atan2((float)sinr_cosp, (float)cosr_cosp);

        // pitch / y
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (Mathf.Abs((float)sinp) >= 1)
        {
            int sign = 0;
            if(sinp >= 0){
                sign = 1;
            }else{
                sign=-1;
            }
            angles.y = Mathf.PI / 2*sign;
        }
        else
        {
            angles.y = (float)Mathf.Asin((float)sinp);
        }

        // yaw / z
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.z = (float)Mathf.Atan2((float)siny_cosp, (float)cosy_cosp);

        return angles;
    }

}
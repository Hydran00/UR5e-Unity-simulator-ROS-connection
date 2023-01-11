using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using UnityEngine;

public class CupPositionController : MonoBehaviour
{
    public GameObject cup;
    // Start is called before the first frame update
    void Start()
    {   
        ROSConnection.GetOrCreateInstance().Subscribe<PoseStampedMsg>("/cup_position", callback);
    }
    // Update is called once per frame
    void Update()
    {   
    }
    void callback(PoseStampedMsg pose){
       // Vector3<FLU> rosPos2;
        //Update position
        Vector3 pos = new Vector3((float)pose.pose.position.x,
        (float)pose.pose.position.y,(float)pose.pose.position.z);
        pos = ConvertPositionToUnity(pos);
        cup.transform.position=pos;

        //Update rotation
        Quaternion rot = new Quaternion ((float)pose.pose.orientation.x,(float)pose.pose.orientation.y,
        (float)pose.pose.orientation.z,(float)pose.pose.orientation.w);
        rot = ConvertRotationToUnity(rot);
        rot.x = (float)pose.pose.orientation.x;
        rot.y = (float)pose.pose.orientation.y;
        rot.z = (float)pose.pose.orientation.z;
        rot.w = (float)pose.pose.orientation.w;
        cup.transform.rotation=rot;
    }

    Vector3 ConvertPositionToUnity(Vector3 pos){
        return new Vector3(-pos.x,pos.z,pos.y);
    }
    Quaternion ConvertRotationToUnity(Quaternion rot){
        return new Quaternion(rot.y,-rot.z,-rot.x,rot.w);
    }
}

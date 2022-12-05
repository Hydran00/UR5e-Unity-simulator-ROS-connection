using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using EigenCore.Core.Dense;
public class test_eigen : MonoBehaviour
{

    public GameObject Target;
    // Start is called before the first frame update
    void Start()
    {
            
    }

    // Update is called once per frame
    void Update()
    {
        MatrixXD v = MatrixXD.Identity(6);
        MatrixXD a = MatrixXD.Identity(6);
        v.Set(0,1, 5);
        a.Set(0,1,v.Get(0,1)/100);
        Rigidbody rb = Target.GetComponent<Rigidbody>();
        Vector3 vel = new Vector3 ((float)a.Get(0,1),0,0);
        rb.velocity = vel;
    }
}

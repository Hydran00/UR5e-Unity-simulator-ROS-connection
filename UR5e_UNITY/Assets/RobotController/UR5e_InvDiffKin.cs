using System.Collections;
using System.Collections.Generic;
using EigenCore.Core.Dense;
using EigenCore.Eigen;
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class UR5e_InvDiffKin
{
    public UR5e_Forward FK;


    private ArticulationBody[] JointList;

    private GameObject target;

    //store old position and rotation for differential computation
    //of linear and angular velocity 
    private Vector3 old_pos;
    private Quaternion old_rot;

    //convergence threshold for linear and angular deltas
    private const float epsilon_lin = 0.001f;
    private const float epsilon_rot = 0.05f;

    //vector to store velocity
    private VectorXD vel;

    //matrix to store forward kinematics results
    private MatrixXD fk_res;
    
    private GameObject ur5e;

    //define convergence parameter
    private const double linear_K= 1;
    private const double angular_K= 0.3;


    //z offset
    private const float z_offset = 1.79f;

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    //Link names
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

    
    //Constructor
    public UR5e_InvDiffKin(GameObject target, GameObject ur5e)
    {
        this.ur5e = ur5e;
        this.old_pos = target.transform.position;
        this.old_rot = target.transform.rotation;
        this.target = target;
        FK = new UR5e_Forward();
        m_JointArticulationBodies = new UrdfJointRevolute[6];
        var linkName = string.Empty;
        for (var i = 0; i < 6; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = ur5e.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
        }
    }

    //Compute inverse differential kinematics
    public MatrixXD ComputeInvDiffKin()
    {
        MatrixXD TH = MatrixXD.Zeros(1, 6);
        VectorXD qk = VectorXD.Zeros(6);

        
        //get current joint angles value
        for(int j=0;j<6;j++)
        {
            qk.Set(j, m_JointArticulationBodies[j].GetPosition());
        }

        //computing EE position for evaluating desired velocity
        fk_res = FK.forward(qk);

        //compute distance from target's position for error reduction
        Vector3 e_o =
            (target.transform.position - old_pos);
        
        //compute distance from target's orientation for error reduction
        Quaternion delta_rot =old_rot  * Quaternion.Inverse(target.transform.rotation);
        Vector3 eulerRot = new Vector3(
                Mathf.DeltaAngle(0, -delta_rot.eulerAngles.x),
                Mathf.DeltaAngle(0, -delta_rot.eulerAngles.y),
                Mathf.DeltaAngle(-180, delta_rot.eulerAngles.z));
        eulerRot *= Mathf.Deg2Rad;
        Vector3 e_w = eulerRot;

        //update old values
        old_pos = new Vector3((float)-fk_res.Get(0,3),z_offset-(float)fk_res.Get(2,3),(float)fk_res.Get(1,3));
        old_rot = QuaternionFromMatrix(fk_res);
        //unity uses left hand coordinates
        old_rot = ConvertToUnity(old_rot);

        //convergence criteria for avoiding loops
        if(Mathf.Abs(e_o.x)<epsilon_lin && Mathf.Abs(e_o.y)<epsilon_lin && Mathf.Abs(e_o.z)<epsilon_lin
        && Mathf.Abs(e_w.x)<epsilon_rot && Mathf.Abs(e_w.y)<epsilon_rot && Mathf.Abs(e_w.z)<epsilon_rot)
        {
            for(int i=0;i<1;i++){
                for(int j=0;j<6;j++){
                    TH.Set(i,j,qk.Get(j));
                }
            }
            return TH;
        }

        //creating velocity vector for each axis
        vel = new VectorXD(new double[] {-e_o.x,e_o.z,-e_o.y,-e_w.x,e_w.z,-e_w.y});
        //Debug.Log(Mathf.Sqrt(e_o.x*e_o.x+ e_o.z*e_o.z+ e_o.y*e_o.y));

        //instantiate vector for updating new joint angles
        VectorXD qk1 = VectorXD.Zeros(6);

        //Define number of iteration
        int step_num = 1;
        double alpha = 10*Time.fixedDeltaTime;

        //computing joint in time wrt past performance in computing TH
        for (int step = 0; step < step_num; step++)
        {
            //compute Jacobian Matrix with current joint angles
            MatrixXD Jac = ComputeJacobian(qk);
           
            //Compute Jacobian Inverse
            MatrixXD JacInv = inverse(Jac);//Jac.Inverse();

            //COMPUTING qk1 = qk + alpha * JacInv  * K * e

            //compute alpha * K * e
            vel.Set(0, alpha * vel.Get(0) * linear_K);
            vel.Set(1, alpha * vel.Get(1) * linear_K);
            vel.Set(2, alpha * vel.Get(2) * linear_K);
            vel.Set(3, alpha * vel.Get(3) * angular_K);
            vel.Set(4, alpha * vel.Get(4) * angular_K);
            vel.Set(5, alpha * vel.Get(5) * angular_K);

            //Matrix multiplication (6x6 * 6x1)
            double temp = 0;
            for (int i = 0; i < 6; i++)
            {
                temp = 0;
                temp += JacInv.Get(i, 0) * vel.Get(0);
                temp += JacInv.Get(i, 1) * vel.Get(1);
                temp += JacInv.Get(i, 2) * vel.Get(2);
                temp += JacInv.Get(i, 3) * vel.Get(3);
                temp += JacInv.Get(i, 4) * vel.Get(4);
                temp += JacInv.Get(i, 5) * vel.Get(5);

                //compute qk1
                qk1.Set(i, qk.Get(i) + temp);
            }

            //adding new computed row to the trajectory (q = [q; qk1])
            for (int i = 0; i < 6; i++)
            {
                TH.Set(step, i, qk1.Get(i));
            }

            //updating qk with qk1
            qk = qk1;
        }
        return TH;
    }

    private MatrixXD ComputeJacobian(VectorXD qk)
    {
        float[] A = new float[] { 0f, -0.425f, -0.3922f, 0f, 0f, 0f };
        float[] D = new float[] { 0.1625f, 0f, 0f, 0.1333f, 0.0997f, 0.16f+0.0996f };

        float A1 = A[0];
        float A2 = A[1];
        float A3 = A[2];
        float A4 = A[3];
        float A5 = A[4];
        float A6 = A[5];
        float D1 = D[0];
        float D2 = D[1];
        float D3 = D[2];
        float D4 = D[3];
        float D5 = D[4];
        float D6 = D[5];

        float th1 = (float) qk.Get(0);
        float th2 = (float) qk.Get(1);
        float th3 = (float) qk.Get(2);
        float th4 = (float) qk.Get(3);
        float th5 = (float) qk.Get(4);
        float th6 = (float) qk.Get(5);

        float[] J1 =
            new float[] {
                D5 *
                (
                Mathf.Cos(th1) * Mathf.Cos(th5) +
                Mathf.Cos(th2 + th3 + th4) * Mathf.Sin(th1) * Mathf.Sin(th5)
                ) +
                D3 * Mathf.Cos(th1) +
                D4 * Mathf.Cos(th1) -
                A3 * Mathf.Cos(th2 + th3) * Mathf.Sin(th1) -
                A2 * Mathf.Cos(th2) * Mathf.Sin(th1) -
                D5 * Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th1),
                D5 *
                (
                Mathf.Cos(th5) * Mathf.Sin(th1) -
                Mathf.Cos(th2 + th3 + th4) * Mathf.Cos(th1) * Mathf.Sin(th5)
                ) +
                D3 * Mathf.Sin(th1) +
                D4 * Mathf.Sin(th1) +
                A3 * Mathf.Cos(th2 + th3) * Mathf.Cos(th1) +
                A2 * Mathf.Cos(th1) * Mathf.Cos(th2) +
                D5 * Mathf.Sin(th2 + th3 + th4) * Mathf.Cos(th1),
                0f,
                0f,
                0f,
                1f
            };
        float[] J2 =
            new float[] {
                -Mathf.Cos(th1) *
                (
                A3 * Mathf.Sin(th2 + th3) +
                A2 * Mathf.Sin(th2) +
                D5 *
                (
                Mathf.Sin(th2 + th3) * Mathf.Sin(th4) -
                Mathf.Cos(th2 + th3) * Mathf.Cos(th4)
                ) -
                D5 *
                Mathf.Sin(th5) *
                (
                Mathf.Cos(th2 + th3) * Mathf.Sin(th4) +
                Mathf.Sin(th2 + th3) * Mathf.Cos(th4)
                )
                ),
                -Mathf.Sin(th1) *
                (
                A3 * Mathf.Sin(th2 + th3) +
                A2 * Mathf.Sin(th2) +
                D5 *
                (
                Mathf.Sin(th2 + th3) * Mathf.Sin(th4) -
                Mathf.Cos(th2 + th3) * Mathf.Cos(th4)
                ) -
                D5 *
                Mathf.Sin(th5) *
                (
                Mathf.Cos(th2 + th3) * Mathf.Sin(th4) +
                Mathf.Sin(th2 + th3) * Mathf.Cos(th4)
                )
                ),
                A3 * Mathf.Cos(th2 + th3) -
                (D5 * Mathf.Sin(th2 + th3 + th4 + th5)) / 2 +
                A2 * Mathf.Cos(th2) +
                (D5 * Mathf.Sin(th2 + th3 + th4 - th5)) / 2 +
                D5 * Mathf.Sin(th2 + th3 + th4),
                Mathf.Sin(th1),
                -Mathf.Cos(th1),
                0f
            };
        float[] J3 =
            new float[] {
                Mathf.Cos(th1) *
                (
                D5 * Mathf.Cos(th2 + th3 + th4) -
                A3 * Mathf.Sin(th2 + th3) +
                D5 * Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th5)
                ),
                Mathf.Sin(th1) *
                (
                D5 * Mathf.Cos(th2 + th3 + th4) -
                A3 * Mathf.Sin(th2 + th3) +
                D5 * Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th5)
                ),
                A3 * Mathf.Cos(th2 + th3) -
                (D5 * Mathf.Sin(th2 + th3 + th4 + th5)) / 2 +
                (D5 * Mathf.Sin(th2 + th3 + th4 - th5)) / 2 +
                D5 * Mathf.Sin(th2 + th3 + th4),
                Mathf.Sin(th1),
                -Mathf.Cos(th1),
                0f
            };
        float[] J4 =
            new float[] {
                D5 *
                Mathf.Cos(th1) *
                (
                Mathf.Cos(th2 + th3 + th4) +
                Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th5)
                ),
                D5 *
                Mathf.Sin(th1) *
                (
                Mathf.Cos(th2 + th3 + th4) +
                Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th5)
                ),
                D5 *
                (
                Mathf.Sin(th2 + th3 + th4 - th5) / 2 +
                Mathf.Sin(th2 + th3 + th4) -
                Mathf.Sin(th2 + th3 + th4 + th5) / 2
                ),
                Mathf.Sin(th1),
                -Mathf.Cos(th1),
                0
            };
        float[] J5 =
            new float[] {
                -D5 * Mathf.Sin(th1) * Mathf.Sin(th5) -
                D5 *
                Mathf.Cos(th2 + th3 + th4) *
                Mathf.Cos(th1) *
                Mathf.Cos(th5),
                D5 * Mathf.Cos(th1) * Mathf.Sin(th5) -
                D5 *
                Mathf.Cos(th2 + th3 + th4) *
                Mathf.Cos(th5) *
                Mathf.Sin(th1),
                -D5 *
                (
                Mathf.Sin(th2 + th3 + th4 - th5) / 2 +
                Mathf.Sin(th2 + th3 + th4 + th5) / 2
                ),
                Mathf.Sin(th2 + th3 + th4) * Mathf.Cos(th1),
                Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th1),
                -Mathf.Cos(th2 + th3 + th4)
            };
        float[] J6 =
            new float[] {
                0f,
                0f,
                0f,
                Mathf.Cos(th5) * Mathf.Sin(th1) -
                Mathf.Cos(th2 + th3 + th4) * Mathf.Cos(th1) * Mathf.Sin(th5),
                -Mathf.Cos(th1) * Mathf.Cos(th5) -
                Mathf.Cos(th2 + th3 + th4) * Mathf.Sin(th1) * Mathf.Sin(th5),
                -Mathf.Sin(th2 + th3 + th4) * Mathf.Sin(th5)
            };
        MatrixXD J = new MatrixXD(new double[6, 6]);
        for (int i = 0; i < 6; i++)
        {
            J.Set(i, 0, (double) J1[i]);
            J.Set(i, 1, (double) J2[i]);
            J.Set(i, 2, (double) J3[i]);
            J.Set(i, 3, (double) J4[i]);
            J.Set(i, 4, (double) J5[i]);
            J.Set(i, 5, (double) J6[i]);
        }
        return J;
    }

    private static Quaternion QuaternionFromMatrix(MatrixXD m)
    {
        Quaternion q = new Quaternion();
        q.w = Mathf.Sqrt(Mathf.Max(0,1 +(float) m.Get(0, 0) +(float) m.Get(1, 1) +(float) m.Get(2, 2))) /2;
        q.x = Mathf.Sqrt(Mathf.Max(0,1 +(float) m.Get(0, 0) -(float) m.Get(1, 1) -(float) m.Get(2, 2))) /2;
        q.y = Mathf.Sqrt(Mathf.Max(0,1 -(float) m.Get(0, 0) +(float) m.Get(1, 1) -(float) m.Get(2, 2))) /2;
        q.z = Mathf.Sqrt(Mathf.Max(0,1 -(float) m.Get(0, 0) -(float) m.Get(1, 1) +(float) m.Get(2, 2))) /2;
        q.x *= Mathf.Sign(q.x * (float)(m.Get(2, 1) - m.Get(1, 2)));
        q.y *= Mathf.Sign(q.y * (float)(m.Get(0, 2) - m.Get(2, 0)));
        q.z *= Mathf.Sign(q.z * (float)(m.Get(1, 0) - m.Get(0, 1)));
        return q;
    }
    private Quaternion ConvertToUnity(Quaternion input)
    {
        return new Quaternion(-input.x, -input.z, -input.y, input.w);
    }

    private int N = 6;
 



    //The following code is for the computation of the inverse of a matrix

    // Function to get cofactor of A[p,q] in [,]temp. n is current
    // dimension of [,]A
    private void getCofactor(MatrixXD A, MatrixXD temp, int p, int q, int n)
    {
        int i = 0, j = 0;
 
        // Looping for each element of the matrix
        for (int row = 0; row < n; row++)
        {
            for (int col = 0; col < n; col++)
            {
                // Copying into temporary matrix only those element
                // which are not in given row and column
                if (row != p && col != q)
                {
                    temp.Set(i, j++,A.Get(row, col));
    
                    // Row is filled, so increase row index and
                    // reset col index
                    if (j == n - 1)
                    {
                        j = 0;
                        i++;
                    }
                }
            }
        }

    }
    /* Recursive function for finding determinant of matrix.
    n is current dimension of [,]A. */
    private double determinant(MatrixXD A, int n)
        {
        double D = 0; // Initialize result
    
        // Base case : if matrix contains single element
        if (n == 1)
            return A.Get(0, 0);
    
        MatrixXD temp = MatrixXD.Zeros(6,6); // To store cofactors
    
        int sign = 1; // To store sign multiplier
    
        // Iterate for each element of first row
        for (int f = 0; f < n; f++)
        {
            // Getting Cofactor of A[0,f]
            getCofactor(A, temp, 0, f, n);
            D += sign * A.Get(0, f) * determinant(temp, n - 1);
    
            // terms are to be added with alternate sign
            sign = -sign;
        }
        return D;
    }
 
// Function to get adjoint of A[N,N] in adj[N,N].
    private void adjoint(MatrixXD A, MatrixXD adj)
    {
        if (N == 1)
        {
            adj.Set(0, 0, 1);
            return;
        }
    
        // temp is used to store cofactors of [,]A
        int sign = 1;
        MatrixXD temp = MatrixXD.Zeros(6,6);
    
        for (int i = 0; i < N; i++)
        {
            for (int j = 0; j < N; j++)
            {
                // Get cofactor of A[i,j]
                getCofactor(A, temp, i, j, N);
    
                // sign of adj[j,i] positive if sum of row
                // and column indexes is even.
                sign = ((i + j) % 2 == 0)? 1: -1;
    
                // Interchanging rows and columns to get the
                // transpose of the cofactor matrix
                adj.Set(j, i, (sign) * (determinant(temp, N - 1)));
            }
        }
    }
 
// Function to calculate and store inverse, returns false if
// matrix is singular
    private MatrixXD inverse(MatrixXD A)
    {
        MatrixXD inv = MatrixXD.Zeros(6,6);
        // Find determinant of [,]A
        double det = determinant(A, N);
        if (det == 0)
        {
            return inv;
        }
    
        // Find adjoint
        MatrixXD adj = MatrixXD.Zeros(6,6);
        adjoint(A, adj);
    
        // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                inv.Set(i, j,adj.Get(i, j)/det);
    
        return inv;
    }
 
// Generic function to display the matrix. We use it to display
// both adjoin and inverse. adjoin is integer matrix and inverse
// is a float.

}
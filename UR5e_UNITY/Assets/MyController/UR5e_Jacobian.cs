using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class UR5e_Jacobian
{
    // Start is called before the first frame update
    
    public UR5e_Jacobian(float[] Th)
    {  /*     
    float[] A = new float[]{0f, -0.425f, -0.3922f, 0f, 0f, 0f};
    float[] D = new float[]{0.1625f, 0f, 0f, 0.1333f, 0.0997f, 0.0996f};
    
    float A1 = A[0]; float A2 = A[1];float  A3 = A[2];float  A4 = A[3];float  A5 = A[4];float   A6 = A[5];
    float D1 = D[0]; float D2 = D[1];float  D3 = D[2];float  D4 = D[3];float  D5 = D[4];float D6 = D[5];
    
    float th1 = Th[0];
    float th2 = Th[1];
    float th3 = Th[2];
    float th4 = Th[3];
    float th5 = Th[4];
    float th6 = Th[5];
    
    float[] J1 = new float[]{
        D5*(Mathf.Cos(th1)*Mathf.Cos(th5) + Mathf.Cos(th2 + th3 + th4)*Mathf.Sin(th1)*Mathf.Sin(th5)) + D3*Mathf.Cos(th1) + D4*Mathf.Cos(th1) - A3*Mathf.Cos(th2 + th3)*Mathf.Sin(th1) - A2*Mathf.Cos(th2)*Mathf.Sin(th1) - D5*Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th1),
        D5*(Mathf.Cos(th5)*Mathf.Sin(th1) - Mathf.Cos(th2 + th3 + th4)*Mathf.Cos(th1)*Mathf.Sin(th5)) + D3*Mathf.Sin(th1) + D4*Mathf.Sin(th1) + A3*Mathf.Cos(th2 + th3)*Mathf.Cos(th1) + A2*Mathf.Cos(th1)*Mathf.Cos(th2) + D5*Mathf.Sin(th2 + th3 + th4)*Mathf.Cos(th1),
        0f,
        0f,
        0f,
        1f};
    float[] J2 = new float[]{
        -Mathf.Cos(th1)*(A3*Mathf.Sin(th2 + th3) + A2*Mathf.Sin(th2) + D5*(Mathf.Sin(th2 + th3)*Mathf.Sin(th4) - Mathf.Cos(th2 + th3)*Mathf.Cos(th4)) - D5*Mathf.Sin(th5)*(Mathf.Cos(th2 + th3)*Mathf.Sin(th4) + Mathf.Sin(th2 + th3)*Mathf.Cos(th4))),
        -Mathf.Sin(th1)*(A3*Mathf.Sin(th2 + th3) + A2*Mathf.Sin(th2) + D5*(Mathf.Sin(th2 + th3)*Mathf.Sin(th4) - Mathf.Cos(th2 + th3)*Mathf.Cos(th4)) - D5*Mathf.Sin(th5)*(Mathf.Cos(th2 + th3)*Mathf.Sin(th4) + Mathf.Sin(th2 + th3)*Mathf.Cos(th4))),
        A3*Mathf.Cos(th2 + th3) - (D5*Mathf.Sin(th2 + th3 + th4 + th5))/2 + A2*Mathf.Cos(th2) + (D5*Mathf.Sin(th2 + th3 + th4 - th5))/2 + D5*Mathf.Sin(th2 + th3 + th4),
        Mathf.Sin(th1),
        -Mathf.Cos(th1),
        0f};
    float[] J3 = new float[]{
        Mathf.Cos(th1)*(D5*Mathf.Cos(th2 + th3 + th4) - A3*Mathf.Sin(th2 + th3) + D5*Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th5)),
        Mathf.Sin(th1)*(D5*Mathf.Cos(th2 + th3 + th4) - A3*Mathf.Sin(th2 + th3) + D5*Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th5)),
        A3*Mathf.Cos(th2 + th3) - (D5*Mathf.Sin(th2 + th3 + th4 + th5))/2 + (D5*Mathf.Sin(th2 + th3 + th4 - th5))/2 + D5*Mathf.Sin(th2 + th3 + th4),
        Mathf.Sin(th1),
        -Mathf.Cos(th1),
        0f};
    float[] J4 = new float[]{
        D5*Mathf.Cos(th1)*(Mathf.Cos(th2 + th3 + th4) + Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th5)),
        D5*Mathf.Sin(th1)*(Mathf.Cos(th2 + th3 + th4) + Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th5)),
        D5*(Mathf.Sin(th2 + th3 + th4 - th5)/2 + Mathf.Sin(th2 + th3 + th4) - Mathf.Sin(th2 + th3 + th4 + th5)/2),
        Mathf.Sin(th1),
        -Mathf.Cos(th1),
        0};
    float[] J5 =new float []{
        -D5*Mathf.Sin(th1)*Mathf.Sin(th5) - D5*Mathf.Cos(th2 + th3 + th4)*Mathf.Cos(th1)*Mathf.Cos(th5),
        D5*Mathf.Cos(th1)*Mathf.Sin(th5) - D5*Mathf.Cos(th2 + th3 + th4)*Mathf.Cos(th5)*Mathf.Sin(th1),
        -D5*(Mathf.Sin(th2 + th3 + th4 - th5)/2 + Mathf.Sin(th2 + th3 + th4 + th5)/2),
        Mathf.Sin(th2 + th3 + th4)*Mathf.Cos(th1),
        Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th1),
        -Mathf.Cos(th2 + th3 + th4)
        };
    float[] J6 = new float[]{
        0f,
        0f,
        0f,
        Mathf.Cos(th5)*Mathf.Sin(th1) - Mathf.Cos(th2 + th3 + th4)*Mathf.Cos(th1)*Mathf.Sin(th5),
        -Mathf.Cos(th1)*Mathf.Cos(th5) - Mathf.Cos(th2 + th3 + th4)*Mathf.Sin(th1)*Mathf.Sin(th5),
        -Mathf.Sin(th2 + th3 + th4)*Mathf.Sin(th5)
    };
    float[,]J= new float[6,6];
    for(int i=0;i<6;i++){
        J[i,0]=J1[i];
        J[i,1]=J2[i];
        J[i,2]=J3[i];
        J[i,3]=J4[i];
        J[i,4]=J5[i];
        J[i,5]=J6[i];
    }
    
    string output = "";
    for(int z=0;z<6;z++){
        for(int j=0;j<6;j++){
            output+=J[z,j]+" ";
        }
        output+="\n";
    }
    Debug.Log(output);
    */}
}

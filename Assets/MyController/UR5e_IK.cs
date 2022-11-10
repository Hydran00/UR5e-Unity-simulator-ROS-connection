using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
public class UR5e_IKS
{   
    private const double PI = 3.141592653589793238463;
    private const double d1 =  0.1625;
    private const double a2 = -0.42500;
    private const double a3 = -0.39225;
    private const double d4 =  0.1333;
    private const double d5 =  0.0997;
    private const double d6 =  0.0996;

    public int inverse(double[] T, double[] q_sols, double q6_des){
    int num_sols = 0;
    double T02 = -T[0]; double T00 = T[1]; double T01 = T[2]; double T03 = -T[3];  
    double T12 = -T[4]; double T10 =  T[5];double T11 =  T[6]; double T13 = -T[7]; 
    double T22 =  T[8]; double T20 = -T[9]; double T21 = -T[10]; double T23 =  T[11];
    double[] q1 = new double[2];
    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = Mathf.Asin((float)div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }
      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = Mathf.Acos((float)div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }
      else if(d4*d4 > R) {
        return num_sols;
      }
      else {
        double arccos = Mathf.Acos((float)d4 / Mathf.Sqrt((float)R)) ;
        double arctan = Mathf.Atan2((float)-B, (float)A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }
    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////

    double[,] q5 = new double[2,2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*Mathf.Sin((float)q1[i]) - T13*Mathf.Cos((float)q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = Mathf.Acos((float)div);
        q5[i,0] = arccos;
        q5[i,1] = 2.0*PI - arccos;
      }
    }
    ////////////////////////////////////////////////////////////////////////////////
    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = Mathf.Cos((float)q1[i]), s1 = Mathf.Sin((float)q1[i]);
          double c5 = Mathf.Cos((float)q5[i,j]), s5 = Mathf.Sin((float)q5[i,j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = Mathf.Atan2(SIGN(s5)*-(float)(T01*s1 - T11*c1), 
                       SIGN(s5)*(float)(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double[] q2 = new double[2];
          double[] q3 =new double[2];
          double[] q4 =new double[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = Mathf.Cos((float)q6), s6 = Mathf.Sin((float)q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);
          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            // TODO NO SOLUTION
            continue;
          }
          double arccos = Mathf.Acos((float)c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = Mathf.Sin((float)arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = Mathf.Atan2((float)((A*p13y - B*p13x) / denom),(float) ((A*p13x + B*p13y) / denom));
          q2[1] = Mathf.Atan2((float)((A*p13y + B*p13x) / denom),(float) ((A*p13x - B*p13y) / denom));
          double c23_0 = Mathf.Cos((float)(q2[0]+q3[0]));
          double s23_0 = Mathf.Sin((float)(q2[0]+q3[0]));
          double c23_1 = Mathf.Cos((float)(q2[1]+q3[1]));
          double s23_1 = Mathf.Sin((float)(q2[1]+q3[1]));
          q4[0] = Mathf.Atan2((float)(c23_0*x04y - s23_0*x04x), (float)(x04x*c23_0 + x04y*s23_0));
          q4[1] = Mathf.Atan2((float)(c23_1*x04y - s23_1*x04x), (float)(x04x*c23_1 + x04y*s23_1));
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i,j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }

        }
      }
    }
    Debug.Log(q1[0]+" "+q1[1]);
    return num_sols;
    
    }

    double fabs(double x){
        if (SIGN(x)==1){
            return x;
        }else{
            return -x;
        }
    }
    double td(double rad){
      return rad * (180.0/3.141592653589793238463);
    }
    const double ZERO_THRESH = 0.00000001;
    int SIGN(double x) {
      if(x>0){
          return 1;
      }else{
          if(x==0){
              return 0;
          }else{
              return -1;
          }
      }
    }
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using EigenCore.Core.Dense;
public class UR5e_Forward
{
    public MatrixXD forward(VectorXD Th){
        VectorXD A = new VectorXD(new double[]{0f, -0.425f, -0.3922f, 0f, 0f, 0f});
        VectorXD D = new VectorXD(new double[]{0.1625f, 0f, 0f, 0.1333f, 0.0997f, 0.16f+0.0996f});
        VectorXD alfa = new VectorXD(new double[]{0, Mathf.PI/2, 0, 0, Mathf.PI/2, -Mathf.PI/2}); 
        
        MatrixXD T10f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(0)), -Math.Sin(Th.Get(0)), 0, 0},
            {Math.Sin(Th.Get(0)), Math.Cos(Th.Get(0)), 0, 0},
            {0, 0, 1, D.Get(0)},
            {0, 0, 0, 1}
        });
        MatrixXD T21f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(1)), -Math.Sin(Th.Get(1)), 0, 0},
            {0, 0, -1, 0},
            {Math.Sin(Th.Get(1)), Math.Cos(Th.Get(1)), 0, 0},
            {0, 0, 0, 1}
        });

        MatrixXD T32f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(2)), -Math.Sin(Th.Get(2)), 0, A.Get(1)},
            {Math.Sin(Th.Get(2)), Math.Cos(Th.Get(2)), 0, 0},
            {0, 0, 1, D.Get(2)},
            {0, 0, 0, 1}
        });

        MatrixXD T43f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(3)), -Math.Sin(Th.Get(3)), 0, A.Get(2)},
            {Math.Sin(Th.Get(3)), Math.Cos(Th.Get(3)), 0, 0},
            {0, 0, 1, D.Get(3)},
            {0, 0, 0, 1}
        });

        MatrixXD T54f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(4)), -Math.Sin(Th.Get(4)), 0, 0},
            {0, 0, -1, -D.Get(4)},
            {Math.Sin(Th.Get(4)), Math.Cos(Th.Get(4)), 0, 0},
            {0, 0, 0, 1}
        });

        MatrixXD T65f = new MatrixXD(new double[,]{
            {Math.Cos(Th.Get(5)), -Math.Sin(Th.Get(5)), 0, 0},
            {0, 0, 1, D.Get(5)},
            {-Math.Sin(Th.Get(5)), -Math.Cos(Th.Get(5)), 0, 0},
            {0, 0, 0, 1}
        });
        MatrixXD T = MultiplyMatrix(T10f,T21f);
        T = MultiplyMatrix(T,T32f);
        T = MultiplyMatrix(T,T43f);
        T = MultiplyMatrix(T,T54f);
        T = MultiplyMatrix(T,T65f); 
        return T;
        }
    public MatrixXD MultiplyMatrix(MatrixXD A, MatrixXD B)
    {
        int rA = 4;
        int cA = 4;
        int rB = 4;
        int cB = 4;
        if (cA != rB)
        {
            Debug.Log("Matrixes can't be multiplied!!");
            return MatrixXD.Zeros(6,6);
        }
        else
        {
            double temp = 0;
            MatrixXD kHasil = new MatrixXD(new double[rA,cB]);
            for (int i = 0; i < rA; i++)
            {
                for (int j = 0; j < cB; j++)
                {
                    temp = 0;
                    for (int k = 0; k < cA; k++)
                    {
                        temp += A.Get(i, k) * B.Get(k, j);
                    }
                    kHasil.Set(i, j,temp);
                }
            }
            return kHasil;
        }
    }
    }  
      
    


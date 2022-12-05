﻿using System.Runtime.InteropServices;
using System.Security;

namespace EigenCore.Eigen
{
    internal static unsafe class ThunkDenseEigen
    {
        internal const string NativeThunkEigenPath = "eigen_core";

        #region Vectors

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double ddot_([In] double* firstVector, [In] double* secondVector, int length);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dadd_([In] double* firstVector, [In] double* secondVector, int length, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dscale_([In] double* firstVector, double scale, int length, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dvnorm_([In] double* firstVector, int length);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dvsquared_norm_([In] double* firstVector, int length);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dvlp1_norm_([In] double* firstVector, int length);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dvlpinf_norm_([In] double* firstVector, int length);

        #endregion Vectors

        #region Matrices

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dminus_([In] double* firstMatrix, int row1, int col1, [In] double* secondMatrix, int row2, int col2, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dmult_([In] double* firstMatrix, int row1, int col1, [In] double* secondMatrix, int row2, int col2, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dnorm_([In] double* firstMatrix, int row1, int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dsquared_norm_([In] double* firstMatrix, int row1, int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dlp1_norm_([In] double* firstMatrix, int row1, int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dlpinf_norm_([In] double* firstMatrix, int row1, int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dtransp_([In] double* firstMatrix, int row1, int col1, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dmultt_([In] double* firstMatrix, int row1, int col1, [In] double* secondMatrix, int row2, int col2, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dmultv_([In] double* firstMatrix, int row1, int col1, [In] double* vector, int length, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void da_multt_([In] double* firstMatrix, int row1, int col1, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void da_tmult_([In] double* firstMatrix, int row1, int col1, [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dtrace_([In] double* firstMatrix, int row1, int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double deigenvalues_(
            [In] double* firstMatrix,
            int size,
            [Out] double* out_real_eigen,
            [Out] double* out_imag_eigen,
            [Out] double* out_real_eigenvectors,
            [Out] double* out_image_eigenvectors);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dselfadjoint_eigenvalues_(
            [In] double* firstMatrix,
            int size,
            [Out] double* out_real_eigen,
            [Out] double* out_real_eigenvectors);

        /// <summary>
        /// A = X + X^T
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="size"></param>
        /// <param name="secondMatrix"></param>
        /// <param name="vout"></param>
        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dxplusxt_([In] double* firstMatrix, int size, [Out] double* vout);

        /// <summary>
        /// A = X + Y
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="row1"></param>
        /// <param name="col1"></param>
        /// <param name="secondMatrix"></param>
        /// <param name="row2"></param>
        /// <param name="col2"></param>
        /// <param name="vout"></param>

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dxplusa_([In] double* firstMatrix, int row1, int col1, [In] double* secondMatrix, int row2, int col2, [Out] double* vout);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="row1"></param>
        /// <param name="col1"></param>
        /// <param name="vout"></param>
        /// <param name="sout"></param>
        /// <param name="uout"></param>
        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsvd_([In] double* firstMatrix, int row1, int col1,
            [Out] double* vout,
            [Out] double* sout,
            [Out] double* uout);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="row1"></param>
        /// <param name="col1"></param>
        /// <param name="vout"></param>
        /// <param name="sout"></param>
        /// <param name="uout"></param>
        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsvd_leastsquares_([In] double* firstMatrix, int row1, int col1,
            [In] double* rhs,
            [Out] double* uout);

        /// <summary>
        /// 
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="row1"></param>
        /// <param name="col1"></param>
        /// <param name="vout"></param>
        /// <param name="sout"></param>
        /// <param name="uout"></param>
        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsvd_bdcSvd_([In] double* firstMatrix, int row1, int col1,
            [Out] double* vout,
            [Out] double* sout,
            [Out] double* uout);


        /// <summary>
        /// 
        /// </summary>
        /// <param name="firstMatrix"></param>
        /// <param name="row1"></param>
        /// <param name="col1"></param>
        /// <param name="vout"></param>
        /// <param name="sout"></param>
        /// <param name="uout"></param>
        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsvd_bdcSvd__leastsquares_([In] double* firstMatrix, int row1, int col1,
            [In] double* rhs,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsolve_colPivHouseholderQr_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [In] double* rhs,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsolve_partialPivLU_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [In] double* rhs,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsolve_fullPivLu_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [In] double* rhs,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsolve_llt_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [In] double* rhs,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dsolve_ldlt_(
             [In] double* firstMatrix,
             int row1,
             int col1,
             [In] double* rhs,
             [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dnormal_equations__leastsquares_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [In] double* rhs,
            [Out] double* uout);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double ddeterminant_(
            [In] double* firstMatrix,
            int row1,
            int col1);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dinverse_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [Out] double* uout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double dabsolute_error_(
            [In] double* m1,
            int row,
            int col,
            [In] double* v1,
            [In] double* v2);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double drelative_error_(
            [In] double* m1,
            int row,
            int col,
            [In] double* v1,
            [In] double* v2);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dhouseholderQR_(
            [In] double* firstMatrix,
            int row1,
            int col1,
            [Out] double* q,
            [Out] double* r);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dcolPivHouseholderQR_(
                                [In] double* firstMatrix,
                                int row1,
                                int col1,
                                [Out] double* q,
                                [Out] double* r,
                                [Out] double* p);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void dfullPivLU_(
                        [In] double* firstMatrix,
                        int row1,
                        int col1,
                        [Out] double* l,
                        [Out] double* u,
                        [Out] double* p,
                        [Out] double* q);

        #endregion Matrices
    }
}

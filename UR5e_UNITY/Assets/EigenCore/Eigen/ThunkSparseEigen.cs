﻿using System.Runtime.InteropServices;
using System.Security;

namespace EigenCore.Eigen
{
    public static unsafe class ThunkSparseEigen
    {
        internal const string NativeThunkEigenPath = "eigen_core";

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_conjugateGradient_(
            int row,
            int col,
            int nnz,
            int maxIterations,
            double tolerance,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values,
            [In] double* inrhs,
            [In] int size,
            [Out] double* vout,
            [Out] int* iterations,
            [Out] double* error);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_biCGSTAB_(
             int row,
             int col,
             int nnz,
             int maxIterations,
             double tolerance,
             [In] int* outerIndex,
             [In] int* innerIndex,
             [In] double* values,
             [In] double* inrhs,
             [In] int size,
             [Out] double* vout,
             [Out] int* iterations,
             [Out] double* error);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_LeastSquaresConjugateGradient_(
             int row,
             int col,
             int nnz,
             int maxIterations,
             double tolerance,
             [In] int* outerIndex,
             [In] int* innerIndex,
             [In] double* values,
             [In] double* inrhs,
             [In] int size,
             [Out] double* vout,
             [Out] int* iterations,
             [Out] double* error);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_GMRES_(
              int row,
              int col,
              int nnz,
              int maxIterations,
              double tolerance,
              [In] int* outerIndex,
              [In] int* innerIndex,
              [In] double* values,
              [In] double* inrhs,
              [In] int size,
              [Out] double* vout,
              [Out] int* iterations,
              [Out] double* error);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_MINRES_(
              int row,
              int col,
              int nnz,
              int maxIterations,
              double tolerance,
              [In] int* outerIndex,
              [In] int* innerIndex,
              [In] double* values,
              [In] double* inrhs,
              [In] int size,
              [Out] double* vout,
              [Out] int* iterations,
              [Out] double* error);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern bool ssolve_DGMRES_(
              int row,
              int col,
              int nnz,
              int maxIterations,
              double tolerance,
              [In] int* outerIndex,
              [In] int* innerIndex,
              [In] double* values,
              [In] double* inrhs,
              [In] int size,
              [Out] double* vout,
              [Out] int* iterations,
              [Out] double* error);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void sadd_(
            int row,
            int col,
            int nnz1,
            [In] int* outerIndex1,
            [In] int* innerIndex1,
            [In] double* values1,
            int nnz2,
            [In] int* outerIndex2,
            [In] int* innerIndex2,
            [In] double* values2,
            [Out] int* nnz,
            [Out] int* outerIndex,
            [Out] int* innerIndex,
            [Out] double* values);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void sminus_(
            int row,
            int col,
            int nnz1,
            [In] int* outerIndex1,
            [In] int* innerIndex1,
            [In] double* values1,
            int nnz2,
            [In] int* outerIndex2,
            [In] int* innerIndex2,
            [In] double* values2,
            [Out] int* nnz,
            [Out] int* outerIndex,
            [Out] int* innerIndex,
            [Out] double* values);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void smult_(
           int row,
           int col,
           int nnz1,
           [In] int* outerIndex1,
           [In] int* innerIndex1,
           [In] double* values1,
           int nnz2,
           [In] int* outerIndex2,
           [In] int* innerIndex2,
           [In] double* values2,
           [Out] int* nnz,
           [Out] int* outerIndex,
           [Out] int* innerIndex,
           [Out] double* values);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void smultv_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [In] double* v1,
           int length,
           [Out] double* vout);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void stranspose_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [Out] int* outerIndexout,
           [Out] int* innerIndexout,
           [Out] double* valuesout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void ssolve_simplicialLLT_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [In] double* inrhs,
           [In] int size,
           [Out] double* vout);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void ssolve_simplicialLDLT_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [In] double* inrhs,
           [In] int size,
           [Out] double* vout);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void ssolve_sparseLU_(
            int row,
            int col,
            int nnz,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values,
            [In] double* inrhs,
            [In] int size,
            [Out] double* vout);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void ssolve_sparseQR_(
            int row,
            int col,
            int nnz,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values,
            [In] double* inrhs,
            [In] int size,
            [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern void snormal_equations__leastsquares_sparselu_(
            int row,
            int col,
            int nnz,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values,
            [In] double* inrhs,
            [In] int size,
            [Out] double* vout);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double snorm_(
            int row,
            int col,
            int nnz,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double ssquaredNorm_(
            int row,
            int col,
            int nnz,
            [In] int* outerIndex,
            [In] int* innerIndex,
            [In] double* values);

        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double srelative_error_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [In] double* rhs,
           [In] double* x);


        [DllImport(NativeThunkEigenPath), SuppressUnmanagedCodeSecurity]
        public static extern double sabsolute_error_(
           int row,
           int col,
           int nnz,
           [In] int* outerIndex,
           [In] int* innerIndex,
           [In] double* values,
           [In] double* rhs,
           [In] double* x);
    }
}
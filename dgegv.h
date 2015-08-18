/*
  This file has my implementation of the LAPACK routine dsygv
  for C++.  This program solves for the eigenvalues and, if
  desired, the eigenvectors for a symmetric, generalized eigenvalue
  problem of the form Ax = lBx, where A and B are two symmetric
  matrices, x is an eigenvector, and l is an eigenvalue.  The
  program assumes that the upper triangles of the two matrices
  are stored.

  There are two function calls defined in this header, of the
  forms

    void dsygv(double **A, double **B, int n, double *E)
    void dsygv(double **A, double **B, int n, double *E, double **Evecs)

    A: the n by n matrix from the left hand side of the equation
    B: the n by n matrix from the right hand side of the equation
    n: the order of the square matrices A and B
    E: an n-element array to hold the eigenvalues l
    Evecs: an n by n matrix to hold the eigenvectors of the
           problem, if they are requested.

  The function call is defined twice, so that whether or not
  eigenvectors are called for the proper version is called.

  Scot Shaw
  28 September 1999
*/

#include <math.h>
#include <string.h>

extern "C" void dgegv_(char *jobvl, char *jobvr, int *n,
               double *a, int *lda, double *b, int *ldb,
               double *alphar, double *aplhai, double *beta,
               double *vl, int *ldvl,
               double *vr, int *ldvr, double *work,
               int *lwork, int *info);

#define ZERO_EPS 1e-5

void dgegv(double *A, double *B, int n, double *eigenvalues, double *eigenvectors, int &nreal)
{
  char jobvl = 'N', jobvr = 'V';

  int lda, ldb, lwork, info, ldvl, ldvr;
  double *a, *b, *work, *alphar, *alphai, *beta;
  alphar = new double[n];
  alphai = new double[n];
  beta = new double[n];

  lda = n; // The leading dimension of the matrix A
  ldb = n; // The leading dimension of the matrix B

  ldvl = 1;
  ldvr = n;

  lwork = (8*n+1) * 5;
  work = new double[lwork]; /* The work array to be used by dsygv and
                   its size. */

  double *vr = new double[n * n];
  memset(vr, 0, n * n * sizeof(double));

  a = new double[n * n];
  b = new double[n * n];
  memcpy(a, A, n * n * sizeof(double));
  memcpy(b, B, n * n * sizeof(double));

  dgegv_(&jobvl, &jobvr, &n, a, &lda, b, &ldb, alphar, alphai, beta, NULL, &ldvl, vr, &ldvr, work, &lwork, &info);

  /*if ((info==0)&&(work[0]>lwork))
    cout << "The pre-set lwork value was sub-optimal for the job that\n"
     << "you gave dsygv.  The used value was " << lwork
     << " whereas " << work[0] << " is optimal.\n";*/

  // get only finite real eigenvalues and corresponding eigenvectors
  nreal = 0;
  for (int i = 0; i < n; i++) {
      //std::cout << "(" << alphar[i] / beta[i] << "," << alphai[i] / beta[i] << "),";
      if (fabs(beta[i]) > 1e-5 && fabs(alphai[i]) < 1e-5) {
          if (eigenvalues)
            eigenvalues[nreal] = alphar[i] / beta[i];
          if (eigenvectors)
            memcpy(eigenvectors + nreal * n, vr + i * n, n * sizeof(double));
          nreal++;
      }
  }

  delete a;
  delete b;
  delete work;
}

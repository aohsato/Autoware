
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "generate_poly_trajectory.h"
#include <eigen3/Eigen/Dense>

#define NUM_ELEMENT (2)

GeneratePolyTrajectory::GeneratePolyTrajectory(void)
{
}

void GeneratePolyTrajectory::solveLinearEquations(Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& x)
{
  // if size(A,1) == size(A,2)         % A is square

  double absDiff = 0.01;

  if (A.isApprox(static_cast<Eigen::MatrixXd>(A.triangularView<Eigen::Lower>()), absDiff))
  {
    // A Is Lower Triangular
    x = A.triangularView<Eigen::Lower>().solve(b);  // This is a simple forward substitution on b
  }
  else if (A.isApprox(static_cast<Eigen::MatrixXd>(A.triangularView<Eigen::Upper>()), absDiff))
  {
    // A is upper triangular
    x = A.triangularView<Eigen::Upper>().solve(b);  // This is a simple backward substitution on b
  }
  else
  {
    if (A.isApprox(A.transpose(), absDiff))
    {
      // A is symmetric
      Eigen::LLT<Eigen::MatrixXd> lltOfA(A);
      if (lltOfA.info() == Eigen::NumericalIssue)
      {
        // A is symmetric positive definite
        Eigen::MatrixXd R = lltOfA.matrixU();
        Eigen::VectorXd y = static_cast<Eigen::MatrixXd>(R.transpose()).colPivHouseholderQr().solve(b);
        x = R.colPivHouseholderQr().solve(y);  // a forward and a backward substitution
        return;
      }
    }
    // general, square A

    Eigen::PartialPivLU<Eigen::MatrixXd> lu(A);

    Eigen::VectorXd y = static_cast<Eigen::MatrixXd>(lu.matrixLU().triangularView<Eigen::UnitLower>())
                            .colPivHouseholderQr()
                            .solve(lu.permutationP() * b);  // lu.permutationP() may need to be transposed
    x = static_cast<Eigen::MatrixXd>(lu.matrixLU().triangularView<Eigen::Upper>())
            .colPivHouseholderQr()
            .solve(y);  // a forward and a backward substitution
  }
  // else                              % A is rectangular
  //  [Q,R] = qr(A);
  //  x = R \ (Q' * b);
  // end
}

void GeneratePolyTrajectory::startComputeQuarticPolynomial(double s0, double dts0, double ddts0, double tf, double dtsf,
                                                           double ddtsf,
                                                           std::vector<std::vector<double> >& resultTrajectory,
                                                           std::vector<std::vector<double> >& resultParam,
                                                           double tickTime)

{
  resultTrajectory.clear();

  // rename inputs
  double xx0 = s0;
  double vv0 = dts0;
  double aa0 = ddts0;
  double xxf = 0;  // undefined
  double vvf = dtsf;
  double aaf = ddtsf;

  double cc2 = aa0;
  double cc3 = vv0;
  double cc4 = xx0;
  double tf2 = tf * tf;
  double tf3 = tf2 * tf;
  double tf4 = tf3 * tf;

  // define A matrix
  Eigen::MatrixXd A(2, 2);
  A << 1 / 2. * tf2, tf, 1 / 6. * tf3, 1 / 2. * tf2;

  // define b vector
  Eigen::VectorXd b(2);
  double tmp1 = aaf - cc2;
  double tmp2 = vvf - cc2 * tf - cc3;
  b << tmp1, tmp2;

  // inverse computation
  Eigen::VectorXd constant(2);
  try
  {
    solveLinearEquations(A, b, constant);
  }
  catch (...)
  {
    ROS_ERROR("solver error");
    return;
  }
  double cc0 = constant(0);
  double cc1 = constant(1);
  double sumjjt = 0.0;

  for (double t = 0.0; t < tf; t += tickTime)
  {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;

    double xxt = 1 / 24.0 * cc0 * t4 + 1 / 6.0 * cc1 * t3 + 1 / 2.0 * cc2 * t2 + cc3 * t + cc4;
    double vvt = 1 / 6.0 * cc0 * t3 + 1 / 2.0 * cc1 * t2 + cc2 * t + cc3;
    double aat = 1 / 2.0 * cc0 * t2 + cc1 * t + cc2;
    double jjt = cc0 * t + cc1;
    sumjjt += fabs(jjt);

    std::vector<double> resultPoint(6);
    resultPoint[0] = t;
    resultPoint[1] = xxt;
    resultPoint[2] = vvt;
    resultPoint[3] = aat;
    resultPoint[4] = jjt;
    resultPoint[5] = sumjjt;

    resultTrajectory.push_back(resultPoint);
  }

  std::vector<double> paramVec;
  paramVec.push_back(cc0);
  paramVec.push_back(cc1);
  paramVec.push_back(cc2);
  paramVec.push_back(cc3);
  paramVec.push_back(cc4);
  resultParam.push_back(paramVec);
}

void GeneratePolyTrajectory::startComputeQuinticPolynomial(double d0, double dtd0, double ddtd0, double tf, double df,
                                                           double dtdf, double ddtdf,
                                                           std::vector<std::vector<double> >& resultTrajectory,
                                                           std::vector<std::vector<double> >& resultParam,
                                                           double tickTime)
{
  double amplitude = 1.0;
  GeneratePolyTrajectory::startComputeQuinticPolynomialWithAmplitude(
      d0, dtd0, ddtd0, tf, df, dtdf, ddtdf, resultTrajectory, resultParam, tickTime, amplitude);
}

void GeneratePolyTrajectory::startComputeQuinticPolynomialWithAmplitude(
    double d0, double dtd0, double ddtd0, double tf, double df, double dtdf, double ddtdf,
    std::vector<std::vector<double> >& resultTrajectory, std::vector<std::vector<double> >& resultParam,
    double tickTime, double amplitude)
{
  resultTrajectory.clear();

  // rename inputs
  double xx0 = d0;
  double vv0 = dtd0;
  double aa0 = ddtd0;
  double xxf = df;
  double vvf = dtdf;
  double aaf = ddtdf;

  double cc3 = aa0;
  double cc4 = vv0;
  double cc5 = xx0;
  double tf2 = tf * tf;
  double tf3 = tf2 * tf;
  double tf4 = tf3 * tf;
  double tf5 = tf4 * tf;

  // define A matrix
  Eigen::MatrixXd A(3, 3);
  A << 1 / 6. * tf3, 1 / 2. * tf2, tf, 1 / 24. * tf4, 1 / 6. * tf3, 1 / 2. * tf2, 1 / 120. * tf5, 1 / 24. * tf4,
      1 / 6. * tf3;

  // define b vector
  Eigen::VectorXd b(3);
  double tmp1 = aaf - cc3;
  double tmp2 = vvf - cc3 * tf - cc4;
  double tmp3 = xxf - 1 / 2. * cc3 * tf2 - cc4 * tf - cc5;
  b << tmp1, tmp2, tmp3;

  // inverse computation
  Eigen::VectorXd constant(3);
  try
  {
    solveLinearEquations(A, b, constant);
  }
  catch (...)
  {
    ROS_ERROR("solver error");
    return;
  }
  double cc0 = constant(0);
  double cc1 = constant(1);
  double cc2 = constant(2);
  double sumjjt = 0.0;

  for (double t = 0.0; t < tf; t += tickTime)
  {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    double xxt = 1 / 120.0 * cc0 * t5 + 1 / 24.0 * cc1 * t4 + 1 / 6.0 * cc2 * t3 + 1 / 2.0 * cc3 * t2 + cc4 * t + cc5;
    double vvt = 1 / 24.0 * cc0 * t4 + 1 / 6.0 * cc1 * t3 + 1 / 2.0 * cc2 * t2 + cc3 * t + cc4;
    double aat = 1 / 6.0 * cc0 * t3 + 1 / 2.0 * cc1 * t2 + cc2 * t + cc3;
    double jjt = 1 / 2.0 * cc0 * t2 + cc1 * t + cc2;
    double xxerror = xxt - xxf;
    double vverror = vvt - vvf;
    double aaerror = aat - aaf;
    sumjjt += fabs(jjt);

    std::vector<double> resultPoint(6);
    resultPoint[0] = t;
    resultPoint[1] = xxt;
    resultPoint[2] = amplitude * vvt;
    resultPoint[3] = amplitude * aat;
    resultPoint[4] = amplitude * jjt;
    resultPoint[5] = amplitude * sumjjt;

    resultTrajectory.push_back(resultPoint);
  }

  std::vector<double> paramVec;
  paramVec.push_back(cc0);
  paramVec.push_back(cc1);
  paramVec.push_back(cc2);
  paramVec.push_back(cc3);
  paramVec.push_back(cc4);
  paramVec.push_back(cc5);
  resultParam.push_back(paramVec);
}

void GeneratePolyTrajectory::computeQuinticValueFromParam(double t, double cc0, double cc1, double cc2, double cc3,
                                                          double cc4, double cc5, double& xxt, double& vvt, double& aat,
                                                          double& jjt)
{
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  xxt = 1 / 120.0 * cc0 * t5 + 1 / 24.0 * cc1 * t4 + 1 / 6.0 * cc2 * t3 + 1 / 2.0 * cc3 * t2 + cc4 * t + cc5;
  vvt = 1 / 24.0 * cc0 * t4 + 1 / 6.0 * cc1 * t3 + 1 / 2.0 * cc2 * t2 + cc3 * t + cc4;
  aat = 1 / 6.0 * cc0 * t3 + 1 / 2.0 * cc1 * t2 + cc2 * t + cc3;
  jjt = 1 / 2.0 * cc0 * t2 + cc1 * t + cc2;
}

void GeneratePolyTrajectory::computeQuarticValueFromParam(double t, double cc0, double cc1, double cc2, double cc3,
                                                          double cc4, double& xxt, double& vvt, double& aat,
                                                          double& jjt)
{
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;

  xxt = 1 / 24.0 * cc0 * t4 + 1 / 6.0 * cc1 * t3 + 1 / 2.0 * cc2 * t2 + cc3 * t + cc4;
  vvt = 1 / 6.0 * cc0 * t3 + 1 / 2.0 * cc1 * t2 + cc2 * t + cc3;
  aat = 1 / 2.0 * cc0 * t2 + cc1 * t + cc2;
  jjt = cc0 * t + cc1;
}

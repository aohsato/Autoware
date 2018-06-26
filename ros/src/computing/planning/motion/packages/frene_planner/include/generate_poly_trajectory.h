#ifndef GENERATE_POLY_TRAJECTORY_H
#define GENERATE_POLY_TRAJECTORY_H

#include <eigen3/Eigen/Dense>

// classes and structures
class GeneratePolyTrajectory
{
private:
  static void solveLinearEquations(Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::VectorXd& x);

public:
  GeneratePolyTrajectory(void);

  static void startComputeQuarticPolynomial(double s0, double dts0, double ddts0, double tf, double dtsf, double ddtsf,
                                            std::vector<std::vector<double> >& resultTrajectory,
                                            std::vector<std::vector<double> >& resultParam, double tickTime);
  static void startComputeQuinticPolynomial(double d0, double dtd0, double ddtd0, double tf, double df, double dtdf,
                                            double ddtdf, std::vector<std::vector<double> >& resultTrajectory,
                                            std::vector<std::vector<double> >& resultParam, double tickTime);

  static void startComputeQuinticPolynomialWithAmplitude(double d0, double dtd0, double ddtd0, double tf, double df,
                                                         double dtdf, double ddtdf,
                                                         std::vector<std::vector<double> >& resultTrajectory,
                                                         std::vector<std::vector<double> >& resultParam,
                                                         double tickTime, double amplitude);

  static void computeQuinticValueFromParam(double t, double cc0, double cc1, double cc2, double cc3, double cc4,
                                           double cc5, double& xxt, double& vvt, double& aat, double& jjt);
  static void computeQuarticValueFromParam(double t, double cc0, double cc1, double cc2, double cc3, double cc4,
                                           double& xxt, double& vvt, double& aat, double& jjt);
};

#endif

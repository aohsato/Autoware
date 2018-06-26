#include "speed_model.h"
#include "wall_manager.h"
#include <iterator>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

using namespace std;

SpeedModel::SpeedModel(void)
{
}

void SpeedModel::filterByInvisibleDanger(
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListIn,
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListOut,
    std::vector<BlindCorner>& blindCornerList, double x0, double y0, double yaw, double paramSpeedModelVo,
    double paramSpeedModelDy, double paramSpeedModelAd, double paramSpeedModelAmax, double paramSpeedModelDv)
{
  trajectoryArrayCartesianListOut.clear();
  if (blindCornerList.size() <= 0)
  {
    trajectoryArrayCartesianListOut = trajectoryArrayCartesianListIn;
    return;
  }

  // first make homogeneous matrix list for each blind corners
  std::vector<Eigen::MatrixXd> hMatrix;
  hMatrix.clear();
  std::vector<bool> isLeftList;
  isLeftList.clear();

  for (std::vector<BlindCorner>::iterator itr = blindCornerList.begin(); itr != blindCornerList.end(); itr++)
  {
    Eigen::VectorXd corner(2);
    corner(0) = itr->x;
    corner(1) = itr->y;

    Eigen::VectorXd direction(2);
    direction(0) = itr->vx;
    direction(1) = itr->vy;

    double isLeft;
    isLeft = itr->isLeft;

    Eigen::MatrixXd h(3, 3);
    h << itr->vx, -itr->vy, itr->x, itr->vy, itr->vx, itr->y, 0, 0, 1;

    hMatrix.push_back(h.inverse());
    isLeftList.push_back(isLeft);
  }

  // next evaluate each trajectories
  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = trajectoryArrayCartesianListIn.begin();
       itr1 != trajectoryArrayCartesianListIn.end(); itr1++)
  {
    // do for each trajectories
    std::vector<std::vector<double> > tmp;
    bool isSafe = true;

    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      // do for each points in the trajectory
      double x = (*itr2)[0];
      double y = (*itr2)[1];
      double theta = (*itr2)[3];
      double vtangential = (*itr2)[5];

      double vx = vtangential * cos(theta);
      double vy = vtangential * sin(theta);

      for (int itrh = 0; itrh < hMatrix.size(); itrh++)
      {
        // do for each homogeneous matrix

        // convert a trajectory point(map coordinate) to a blind corner coordinate
        Eigen::VectorXd ph(3);
        ph << x, y, 1.0;
        Eigen::VectorXd pe = hMatrix[itrh] * ph;
        double xe = pe(0);
        double ye = pe(1);

        // convert a velocity vector (map coordinate) to a blind corner coordinate
        Eigen::VectorXd vh(2);
        vh << vx, vy;
        Eigen::VectorXd rh(2, 2);
        rh << hMatrix[itrh](0, 0), hMatrix[itrh](0, 1), hMatrix[itrh](1, 0), hMatrix[itrh](1, 1);
        Eigen::VectorXd ve = rh * vh;

        bool isLeft = isLeftList[itrh];

        double ve0tmp, y0tmp;
        double vxe = ve(0);
        double vye = ve(1);
        double vysafe = computeSafeSpeed(xe, ye, vxe, vye, isLeft, paramSpeedModelVo, paramSpeedModelDy,
                                         paramSpeedModelAd, paramSpeedModelAmax, paramSpeedModelDv, ve0tmp, y0tmp);
        if (isLeft)
        {
          if (ve(1) > vysafe)
          {
            isSafe = false;
            break;
          }
        }
        else
        {
          // right side flip sign
          if (-ve(1) > vysafe)
          {
            isSafe = false;
            break;
          }
        }
      }

      if (!isSafe)
      {
        break;
      }

      tmp.push_back(*itr2);
    }

    if (isSafe)
    {
      trajectoryArrayCartesianListOut.push_back(tmp);
    }
    else
    {
      // filter out
    }
  }
}

double SpeedModel::computeSafeSpeed(double xein, double yein, double vxe, double vye, bool isLeft, double vo, double dy,
                                    double ad, double amax, double dv, double& ve0out, double& y0out)
{
  double normV = sqrt(pow(vxe, 2.0) + pow(vye, 2.0));
  double cosYaw = vxe / normV;
  double sinYaw = vye / normV;

  double xe = xein + dv * cosYaw;
  double ye = yein + dv * sinYaw;

  if (!isLeft)
  {
    dy = -dy;
  }

  double amax2 = pow(amax, 2.0);
  double xe2 = pow(xe, 2.0);
  double xe4 = pow(xe2, 2.0);
  double vo2 = pow(vo, 2.0);
  // dy > 0
  double y0left = (-amax * xe2 - sqrt(amax2 * xe4 + 2 * amax * xe2 * dy * vo2)) / vo2;
  double y0right = (amax * xe2 + sqrt(amax2 * xe4 - 2 * amax * xe2 * dy * vo2)) / vo2;

  double y0 = y0left;
  if (!isLeft)
  {
    y0 = y0right;
  }

  double ve0 = sqrt(2 * amax * fabs(dy - y0));
  double safeSpeed = ve0 + sqrt(2 * ad * fabs(ye - y0));
  if (!isLeft)
  {
    if ((ye - y0) < 0)
    {
      safeSpeed = 1e+09;
    }
  }
  else
  {
    if ((ye - y0) > 0)
    {
      safeSpeed = 1e+09;
    }
  }

  ve0out = ve0;
  y0out = y0;

  return safeSpeed;
}

void SpeedModel::computeSafeSpeedForBlindCorner(BlindCorner& blindCorner,
                                                std::vector<std::vector<std::vector<double> > >& result, double vo,
                                                double dy, double ad, double amax, double dv)
{
  result.clear();

  double xc = blindCorner.x;
  double yc = blindCorner.y;
  double zc = blindCorner.z;

  double vx = blindCorner.vx;
  double vy = blindCorner.vy;
  double vz = blindCorner.vz;

  bool isLeft = blindCorner.isLeft;

  double xeDelta = 0.5;
  double yeDelta = 0.5;
  double xeRange = 10.0;
  double yeRange = 10.0;

  std::vector<std::vector<std::vector<double> > > tmp3;
  for (double xe = 0.0; xe < xeRange; xe += xeDelta)
  {
    std::vector<std::vector<double> > tmp2;
    for (double ye = -yeRange; ye < yeRange; ye += yeDelta)
    {
      double ve0out = 0.0;
      double y0out = 0.0;
      double vxe = 0.0;
      double vye = 1.0;
      if (!isLeft)
      {
        vye = -1.0;
      }

      double vsafe = computeSafeSpeed(xe, ye, vxe, vye, isLeft, vo, dy, ad, amax, dv, ve0out, y0out);

      // to draw inflection points
      double xetmp = xe;
      double yetmp = y0out;
      double vtmp = ve0out;

      Eigen::MatrixXd h(3, 3);
      h << vx, -vy, xc, vy, vx, yc, 0, 0, 1.0;

      Eigen::VectorXd pe(3);
      pe << xetmp, yetmp, 1;
      Eigen::VectorXd peh = h * pe;

      double x2 = peh(0);
      double y2 = peh(1);

      std::vector<double> tmp(21);
      tmp[0] = x2;
      tmp[1] = y2;
      tmp[5] = vtmp;

      tmp2.push_back(tmp);
    }
    tmp3.push_back(tmp2);
  }
  result = tmp3;
}

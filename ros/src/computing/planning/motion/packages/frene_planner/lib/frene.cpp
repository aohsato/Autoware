#include "frene.h"

#include <ros/ros.h>
#include <fstream>
#include <iterator>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
using namespace std;
#include "spline.h"
#include "generate_poly_trajectory.h"
#include "lane_list.h"
#include "trajectory_array.h"

tk::spline _splineX;
tk::spline _splineY;

Frene::Frene(std::vector<std::vector<double> >& laneListIn, double resamplingThreshold,
             std::vector<std::vector<double> >& xyResamplingList)
{
  if (laneListIn.size() <= 0)
  {
    ROS_ERROR("Error. Frene::Frene: laneListIn size must be larger than 0");
    return;
  }

  computeSplineFromXYListWithResamplingDenseData(laneListIn, resamplingThreshold, xyResamplingList);

#if DEBUG
  // for debug
  static bool firstFlag = true;
  if (firstFlag)
  {
    ofstream fout("/tmp/xyResamplingList.txt");
    for (std::vector<std::vector<double> >::iterator itr = xyResamplingList.begin(); itr != xyResamplingList.end();
         itr++)
    {
      fout << (*itr)[0] << " " << (*itr)[1] << std::endl;
    }
    fout.close();
    firstFlag = false;
  }
#endif
};

// xyList x0 y0; x1 y1; ...
// xyInterpList x0 y0; x1' y1'; ...
void Frene::computeSplineFromXYListWithResamplingDenseData(std::vector<std::vector<double> >& xyList,
                                                           double resamplingThreshold,
                                                           std::vector<std::vector<double> >& xyResamplingListReturn)
{
  // resampling
  double xp0 = 0.0;
  double yp0 = 0.0;

  std::vector<std::vector<double> > xyResamplingList;
  xyResamplingList.clear();
  bool isFirstRun = true;
  int count = 0;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xp1 = (*xy)[0];
    double yp1 = (*xy)[1];

    double distance = sqrt(pow(xp1 - xp0, 2.0) + pow(yp1 - yp0, 2.0));

    if (isFirstRun || distance > resamplingThreshold)
    {
      std::vector<double> point;
      point.push_back(xp1);
      point.push_back(yp1);
      xyResamplingList.push_back(point);

      xp0 = xp1;
      yp0 = yp1;
      isFirstRun = false;

      count++;
    }
    else
    {
      // skip
    }
  }

  _totalArcLength = computeSplineFromXYList(xyResamplingList);

  xyResamplingListReturn = xyResamplingList;
}

double Frene::computeSplineFromXYList(std::vector<std::vector<double> >& xyList)
{
  //============================================================================
  // check xy list size
  if (xyList.size() <= 0)
  {
    ROS_ERROR("Erorr: xyList.size() must be larger than0");
    return 0;
  }
  if (xyList[0].size() < 2)
  {
    ROS_ERROR("Erorr: xyList[0].size() must be larger than 2");
    return 0;
  }

  // first compute distance between each points and also total distance
  double xp0 = 0.0;
  double yp0 = 0.0;
  double sumd = 0.0;
  bool isFirstRun = true;
  std::vector<double> dvec;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xp1 = (*xy)[0];
    double yp1 = (*xy)[1];
    double d = sqrt(pow(xp1 - xp0, 2.0) + pow(yp1 - yp0, 2.0));

    if (isFirstRun)
    {
      d = 0.0;
      isFirstRun = false;
    }

    dvec.push_back(d);

    xp0 = xp1;
    yp0 = yp1;
    sumd = sumd + d;
  }

  // compute arc length s and make s vector
  std::vector<double> svec;
  double s0 = 0.0;
  for (std::vector<double>::iterator d = dvec.begin(); d != dvec.end(); d++)
  {
    double s1 = s0 + (*d);
    svec.push_back(s1);
    s0 = s1;
  }

  // make x, y position vector
  std::vector<double> yvec;
  std::vector<double> xvec;
  for (std::vector<std::vector<double> >::iterator xy = xyList.begin(); xy != xyList.end(); xy++)
  {
    double xpoint = (*xy)[0];
    double ypoint = (*xy)[1];

    xvec.push_back(xpoint);
    yvec.push_back(ypoint);
  }

  // now spline interpolation
  _splineX.set_points(svec, xvec);
  _splineY.set_points(svec, yvec);

#if DEBUG
  // for debug
  static bool firstFlag = true;
  if (firstFlag)
  {
    ofstream fout("/tmp/setpoints.txt");
    for (int i = 0; i < svec.size(); i++)
    {
      fout << svec[i] << " " << xvec[i] << " " << yvec[i] << std::endl;
    }
    fout.close();
    firstFlag = false;
  }
#endif

  // return total length
  return sumd;
}

void Frene::convertFromCartesianState(double t, double x, double y, double theta, double vx, double ax, double kx,
                                      double dtTheta, FreneState& freneState, double arcLengthTickMeter)
{
  double s0, d0, k0, dsk0;
  double distanceAbsMin = 1e+14;
  FreneState freneStateMin;

#if DEBUG
  // for debug
  static bool firstFlag = true;
  if (firstFlag)
  {
    ofstream fout("/tmp/lineState.txt");
    for (double s = 0; s < _totalArcLength; s += arcLengthTickMeter)
    {
      LineState lineState;
      computeLineStateAt(s, lineState);
      fout << s << " " << lineState.kr << " " << lineState.dskr << " " << lineState.rr(0) << " " << lineState.rr(1)
           << std::endl;
    }
    fout.close();
    firstFlag = false;
  }
#endif
  for (double s = 0; s < _totalArcLength; s += arcLengthTickMeter)
  {
    LineState lineState;
    computeLineStateAt(s, lineState);

    Eigen::VectorXd rr = lineState.rr;
    Eigen::VectorXd tr = lineState.tr;
    Eigen::VectorXd nr = lineState.nr;
    double thetar = lineState.thetar;
    double kr = lineState.kr;
    Eigen::VectorXd dstr = lineState.tr;
    double dskr = lineState.dskr;

    Eigen::VectorXd p(2);
    p << x, y;
    double d = (p - rr).dot(nr);
    double distance = (p - rr).norm();

    double distanceAbs = fabs(distance);

    Eigen::VectorXd tx(2);
    tx << cos(theta), sin(theta);

    double deltaTheta = theta - thetar;

    double dtd = vx * sin(deltaTheta);
    double dts = cos(deltaTheta) / (1 - kr * d) * vx;
    double dsd = (1 - kr * d) * tan(deltaTheta);

    double ddsd = -(dskr * d + kr * dsd) * tan(deltaTheta) +
                  (1 - kr * d) / (cos(deltaTheta) * cos(deltaTheta)) * (kx * (1 - kr * d) / cos(deltaTheta) - kr);

    Eigen::VectorXd dstx(2);
    dstx << -sin(theta) / dts * dtTheta, cos(theta) / dts * dtTheta;
    Eigen::VectorXd deltadsTrdsTx = dstr - dstx;
    double dsdeltaTheta = atan2(deltadsTrdsTx(1), deltadsTrdsTx(0));

    double ddts =
        cos(deltaTheta) / (1 - kr * d) *
        (ax - dts * dts / cos(deltaTheta) * ((1 - kr * d) * tan(deltaTheta) * dsdeltaTheta - (dskr * d + kr * dsd)));

    double ddtd = ddsd * dts * dts + dsd * ddts;

    if (distanceAbs < distanceAbsMin)
    {
      distanceAbsMin = distanceAbs;
      freneStateMin.td = t;
      freneStateMin.ts = t;
      freneStateMin.d = d;
      freneStateMin.dtd = dtd;
      freneStateMin.ddtd = ddtd;
      freneStateMin.s = s;
      freneStateMin.dts = dts;
      freneStateMin.ddts = ddts;
    }
  }

  freneState = freneStateMin;
}

void Frene::computeLineStateAt(double s, LineState& state)
{
  // read values from list
  double xr = _splineX(s);
  double yr = _splineY(s);
  double sr = s;
  double dsxr = _splineX.deriv(1, s);
  double dsyr = _splineY.deriv(1, s);
  double ddsxr = _splineX.deriv(2, s);
  double ddsyr = _splineY.deriv(2, s);
  double dddsxr = _splineX.deriv(3, s);
  double dddsyr = _splineY.deriv(3, s);

  // convert the values to FreneState
  Eigen::Vector2d rr(2);
  rr << xr, yr;

  Eigen::Matrix2d rot90;
  rot90 << 0, -1, 1, 0;
  double thetar = atan2(dsyr, dsxr);
  double dsThetar = pow(cos(thetar), 2.0) * ((ddsyr * dsxr - dsyr * ddsxr) / pow(dsxr, 2.0));

  Eigen::Vector2d tr;
  tr << dsxr, dsyr;
  tr.normalized();
  Eigen::VectorXd nr = rot90 * tr;
  Eigen::Vector2d dstr;
  dstr << -sin(thetar) * dsThetar, cos(thetar) * dsThetar;

  double dsxr2 = pow(dsxr, 2.0);
  double dsyr2 = pow(dsyr, 2.0);
  double kr = (dsxr * ddsyr - dsyr * ddsxr) * pow(dsxr2 + dsyr2, -3.0 / 2.0);
  double dskr = (ddsxr * ddsyr + dsxr * dddsyr - ddsyr * ddsxr - dsyr * dddsxr) * pow(dsxr2 + dsyr2, -3.0 / 2.0) -
                3 * (dsxr * ddsyr - dsyr * ddsxr) * pow(dsxr2 + dsyr2, -5.0 / 2.0) * (dsxr * ddsxr + dsyr * ddsyr);

  state.rr = rr;
  state.nr = nr;
  state.tr = tr;
  state.dstr = dstr;
  state.dskr = dskr;
  state.kr = kr;
  state.thetar = thetar;
}

void Frene::convertFromFreneState(FreneState& freneState, double& x, double& y, double& theta, double& vx, double& ax,
                                  double& kx, double& dtTheta)
{
  double d = freneState.d;
  double dtd = freneState.dtd;
  double ddtd = freneState.ddtd;
  double s = freneState.s;
  double dts = freneState.dts;
  double ddts = freneState.ddts;

  LineState lineState;
  computeLineStateAt(s, lineState);
  double kr = lineState.kr;
  double dskr = lineState.dskr;
  Eigen::VectorXd dstr = lineState.dstr;
  double thetar = lineState.thetar;
  Eigen::VectorXd rr = lineState.rr;
  Eigen::VectorXd nr = lineState.nr;

  double dsd = dtd / dts;
  double ddsd = (ddtd - dsd * ddts) / pow(dts, 2.0);

  double deltaTheta = atan(dsd / (1 - kr * d));

  double dsTheta = (ddsd + (dskr * d + kr * dsd) * tan(deltaTheta)) * pow(deltaTheta, 2.0) / (1 - kr * d) + kr;
  kx = cos(deltaTheta) / (1 - kr * d) * dsTheta;

  vx = sqrt(pow(1 - kr * d, 2.0) * pow(dts, 2.0) + pow(dtd, 2.0));
  Eigen::VectorXd p = rr + d * nr;

  x = p(0);
  y = p(1);

  theta = deltaTheta + thetar;
  dtTheta = kx * vx;

  double dsdeltaTheta = kx * (1 - kr * d) / cos(deltaTheta) - kr;

  ax = ddts * (1 - kr * d) / cos(deltaTheta) +
       dts * dts / cos(deltaTheta) * ((1 - kr * d) * tan(deltaTheta) * dsdeltaTheta - (dskr * d + kr * dsd));
}

bool Frene::isSmallerThanOrApproxEqual(double val1, double val2, double eps)
{
  if (val1 < val2 || fabs(val1 - val2) < eps)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void Frene::generateLateralAndLongitudinalTrajectoryStopping(
    FreneState& initialState, std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
    std::vector<std::vector<std::vector<double> > >& paramArrayList, double desiredLongSpeed,
    double desiredLongAcceleration, double timeMinSec, double timeMaxSec, double timeDeltaSec,
    double desiredLongPosition, double desiredLongPositionMinus, double desiredLongPositionPlus,
    double desiredLongPositionDelta, double desiredLatPosition, double desiredLatPositionMinus,
    double desiredLatPositionPlus, double desiredLatPositionDelta, double tickTime)
{
  generateLateralAndLongitudinalTrajectoryFollowing(
      initialState, trajectoryArrayList, paramArrayList, desiredLongSpeed, desiredLongAcceleration, timeMinSec,
      timeMaxSec, timeDeltaSec, desiredLongPosition, desiredLongPositionMinus, desiredLongPositionPlus,
      desiredLongPositionDelta, desiredLatPosition, desiredLatPositionMinus, desiredLatPositionPlus,
      desiredLatPositionDelta, tickTime);
}

void Frene::generateLateralAndLongitudinalTrajectoryFollowing(
    FreneState& initialState, std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
    std::vector<std::vector<std::vector<double> > >& paramArrayList, double desiredLongSpeed,
    double desiredLongAcceleration, double timeMinSec, double timeMaxSec, double timeDeltaSec,
    double desiredLongPosition, double desiredLongPositionMinus, double desiredLongPositionPlus,
    double desiredLongPositionDelta, double desiredLatPosition, double desiredLatPositionMinus,
    double desiredLatPositionPlus, double desiredLatPositionDelta, double tickTime)
{
  trajectoryArrayList.clear();

  for (double tf = timeMinSec; tf < timeMaxSec; tf += timeDeltaSec)
  {
    for (double sf = desiredLongPosition + desiredLongPositionMinus;
         isSmallerThanOrApproxEqual(sf, desiredLongPosition + desiredLongPositionPlus, 1e-04);
         sf += desiredLongPositionDelta)
    {
      std::vector<std::vector<double> > trajectoryLongitudinal;
      std::vector<std::vector<double> > paramLongitudinal;

      GeneratePolyTrajectory::startComputeQuinticPolynomial(initialState.s, initialState.dts, initialState.ddts, tf, sf,
                                                            desiredLongSpeed, desiredLongAcceleration,
                                                            trajectoryLongitudinal, paramLongitudinal, tickTime);
      for (double df = desiredLatPosition + desiredLatPositionMinus;
           isSmallerThanOrApproxEqual(df, desiredLatPosition + desiredLatPositionPlus, 1e-04);
           df += desiredLatPositionDelta)
      {
        std::vector<std::vector<double> > trajectoryLateral;
        std::vector<std::vector<double> > paramLateral;
        GeneratePolyTrajectory::startComputeQuinticPolynomial(initialState.d, initialState.dtd, initialState.ddtd, tf,
                                                              df, 0.0, 0.0, trajectoryLateral, paramLateral, tickTime);

        std::vector<std::vector<double> > trajectoryMerge;
        std::vector<std::vector<double> > paramMerge;
        TrajectoryArray::mergeTrajectoryList(trajectoryLongitudinal, trajectoryLateral, trajectoryMerge);
        TrajectoryArray::mergeTrajectoryList(paramLongitudinal, paramLateral, paramMerge);

        trajectoryArrayList.push_back(trajectoryMerge);
        paramArrayList.push_back(paramMerge);
      }
    }
  }
}

void Frene::generateLateralAndLongitudinalTrajectoryVelocityKeeping(
    FreneState& initialState, std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
    std::vector<std::vector<std::vector<double> > >& paramArrayList, double timeMinSec, double timeMaxSec,
    double timeDeltaSec, double desiredSpeed, double desiredSpeedMinus, double desiredSpeedPlus,
    double desiredSpeedDelta, double desiredLatPosition, double desiredLatPositionMinus, double desiredLatPositionPlus,
    double desiredLatPositionDelta, double tickTime)
{
  trajectoryArrayList.clear();
  for (double tf = timeMinSec; tf < timeMaxSec; tf += timeDeltaSec)
  {
    for (double dtsf = desiredSpeed + desiredSpeedMinus;
         isSmallerThanOrApproxEqual(dtsf, desiredSpeed + desiredSpeedPlus, 1e-04); dtsf += desiredSpeedDelta)
    {
      std::vector<std::vector<double> > trajectoryLongitudinal;
      std::vector<std::vector<double> > paramLongitudinal;
      GeneratePolyTrajectory::startComputeQuarticPolynomial(initialState.s, initialState.dts, initialState.ddts, tf,
                                                            dtsf, 0.0, trajectoryLongitudinal, paramLongitudinal,
                                                            tickTime);
      for (double df = desiredLatPosition + desiredLatPositionMinus;
           isSmallerThanOrApproxEqual(df, desiredLatPosition + desiredLatPositionPlus, 1e-04);
           df += desiredLatPositionDelta)
      {
        std::vector<std::vector<double> > trajectoryLateral;
        std::vector<std::vector<double> > paramLateral;
        GeneratePolyTrajectory::startComputeQuinticPolynomial(initialState.d, initialState.dtd, initialState.ddtd, tf,
                                                              df, 0.0, 0.0, trajectoryLateral, paramLateral, tickTime);

        std::vector<std::vector<double> > trajectoryMerge;
        std::vector<std::vector<double> > paramMerge;
        TrajectoryArray::mergeTrajectoryList(trajectoryLongitudinal, trajectoryLateral, trajectoryMerge);
        TrajectoryArray::mergeTrajectoryList(paramLongitudinal, paramLateral, paramMerge);

        trajectoryArrayList.push_back(trajectoryMerge);
        paramArrayList.push_back(paramMerge);
      }
    }
  }
}

void Frene::convertMergedTrajectoryArrayToCartesian(
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayList,
    std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianList)
{
  trajectoryArrayCartesianList.clear();
  trajectoryArrayCartesianList.reserve(trajectoryArrayList.size());
  std::vector<std::vector<double> > trajectoryCartesian;
  std::vector<double> point;
  point.reserve(32);	// over 21

  for (std::vector<std::vector<std::vector<double> > >::iterator itr0 = trajectoryArrayList.begin();
       itr0 != trajectoryArrayList.end(); itr0++)
  {
    trajectoryCartesian.clear();
    trajectoryCartesian.reserve(itr0->size());

    for (std::vector<std::vector<double> >::iterator itr1 = itr0->begin(); itr1 != itr0->end(); itr1++)
    {
      FreneState freneState;
      if (itr1->size() < FreneState::size())
      {
        ROS_ERROR("Error: Merged trajectory size is incorrect");
      }

      freneState.ts = (*itr1)[0];
      freneState.s = (*itr1)[1];
      freneState.dts = (*itr1)[2];
      freneState.ddts = (*itr1)[3];
      freneState.dddts = (*itr1)[4];
      freneState.sumdddts = (*itr1)[5];
      freneState.td = (*itr1)[6];
      freneState.d = (*itr1)[7];
      freneState.dtd = (*itr1)[8];
      freneState.ddtd = (*itr1)[9];
      freneState.dddtd = (*itr1)[10];
      freneState.sumdddtd = (*itr1)[11];

      double x, y, theta, dtTheta, vx, ax, kx;
      convertFromFreneState(freneState, x, y, theta, vx, ax, kx, dtTheta);
      double t = freneState.ts;

      point.clear();
      point.push_back(x);
      point.push_back(y);
      point.push_back(t);
      point.push_back(theta);
      point.push_back(dtTheta);
      point.push_back(vx);
      point.push_back(ax);
      point.push_back(kx);
      point.push_back(freneState.ts);
      point.push_back(freneState.s);
      point.push_back(freneState.dts);
      point.push_back(freneState.ddts);
      point.push_back(freneState.dddts);
      point.push_back(freneState.sumdddts);
      point.push_back(freneState.td);
      point.push_back(freneState.d);
      point.push_back(freneState.dtd);
      point.push_back(freneState.ddtd);
      point.push_back(freneState.dddtd);
      point.push_back(freneState.sumdddtd);

      double centrifugalForce = kx * pow(vx, 2.0);
      point.push_back(centrifugalForce);

      trajectoryCartesian.push_back(point);
    }
    trajectoryArrayCartesianList.push_back(trajectoryCartesian);
  }
}

void Frene::computeInitialStateDirectFromPreviousTrajectoryWithSplineResampling(
    double xCurrent, double yCurrent, std::vector<std::vector<std::vector<double> > >& previousTrajectoryArray,
    FreneState& freneState, double resamplingTickTime)
{
  std::vector<double> xvec;
  std::vector<double> yvec;
  std::vector<double> tvec;
  std::vector<double> thetavec;
  std::vector<double> dtThetavec;
  std::vector<double> vxvec;
  std::vector<double> axvec;
  std::vector<double> kxvec;

  std::vector<double> tsvec;
  std::vector<double> svec;
  std::vector<double> dtsvec;
  std::vector<double> ddtsvec;
  std::vector<double> dddtsvec;
  std::vector<double> tdvec;
  std::vector<double> dvec;
  std::vector<double> dtdvec;
  std::vector<double> ddtdvec;
  std::vector<double> dddtdvec;
  std::vector<double> centrifugalForcevec;

  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = previousTrajectoryArray.begin();
       itr1 != previousTrajectoryArray.end(); itr1++)
  {
    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      if (itr2->size() < 21)
      {
        return;
      }
      double xt = (*itr2)[0];
      double yt = (*itr2)[1];
      double tt = (*itr2)[2];
      double thetat = (*itr2)[3];
      double dtThetat = (*itr2)[4];
      double vxt = (*itr2)[5];
      double axt = (*itr2)[6];
      double kxt = (*itr2)[7];
      double ts = (*itr2)[8];
      double s = (*itr2)[9];
      double dts = (*itr2)[10];
      double ddts = (*itr2)[11];
      double dddts = (*itr2)[12];
      double sumdddts = (*itr2)[13];
      double td = (*itr2)[14];
      double d = (*itr2)[15];
      double dtd = (*itr2)[16];
      double ddtd = (*itr2)[17];
      double dddtd = (*itr2)[18];
      double sumdddtd = (*itr2)[19];
      double centrifugalForce = (*itr2)[20];

      xvec.push_back(xt);
      yvec.push_back(yt);
      tvec.push_back(tt);
      thetavec.push_back(thetat);
      dtThetavec.push_back(dtThetat);
      vxvec.push_back(vxt);
      axvec.push_back(axt);
      kxvec.push_back(kxt);
      tsvec.push_back(ts);
      svec.push_back(s);
      dtsvec.push_back(dts);
      ddtsvec.push_back(ddts);
      dddtsvec.push_back(dddts);
      tdvec.push_back(td);
      dvec.push_back(d);
      dtdvec.push_back(dtd);
      ddtdvec.push_back(ddtd);
      dddtdvec.push_back(ddtd);
      centrifugalForcevec.push_back(centrifugalForce);
    }
  }

  // now spline interpolation
  tk::spline splineX;
  tk::spline splineY;
  tk::spline splineT;
  tk::spline splineTheta;
  tk::spline splineDtTheta;
  tk::spline splineVx;
  tk::spline splineAx;
  tk::spline splineKx;
  tk::spline splineTs;
  tk::spline splineS;
  tk::spline splineDts;
  tk::spline splineDdts;
  tk::spline splineDddts;
  tk::spline splineTd;
  tk::spline splineD;
  tk::spline splineDtd;
  tk::spline splineDdtd;
  tk::spline splineDddtd;
  tk::spline splineCentrifugalForce;

  splineX.set_points(tvec, xvec);
  splineY.set_points(tvec, yvec);
  splineT.set_points(tvec, tvec);
  splineTheta.set_points(tvec, thetavec);
  splineDtTheta.set_points(tvec, dtThetavec);
  splineVx.set_points(tvec, vxvec);
  splineAx.set_points(tvec, axvec);
  splineKx.set_points(tvec, kxvec);
  splineTs.set_points(tvec, tsvec);
  splineS.set_points(tvec, svec);
  splineDts.set_points(tvec, dtsvec);
  splineDdts.set_points(tvec, ddtsvec);
  splineDddts.set_points(tvec, dddtsvec);
  splineTd.set_points(tvec, tdvec);
  splineD.set_points(tvec, dvec);
  splineDtd.set_points(tvec, dtdvec);
  splineDdtd.set_points(tvec, ddtdvec);
  splineDddtd.set_points(tvec, dddtdvec);
  splineCentrifugalForce.set_points(tvec, centrifugalForcevec);

  // find the nearest point in resampling spline
  double tf = tvec.back();
  double distanceMin = 1e+14;
  for (double t0 = 0; t0 < tf; t0 += resamplingTickTime)
  {
    double xt = splineX(t0);
    double yt = splineY(t0);
    double tst = splineTs(t0);
    double st = splineS(t0);
    double dtst = splineDts(t0);
    double ddtst = splineDdts(t0);
    double dddtst = splineDddts(t0);
    double tdt = splineTd(t0);
    double dt = splineD(t0);
    double dtdt = splineDtd(t0);
    double ddtdt = splineDdtd(t0);
    double dddtdt = splineDddtd(t0);

    double distance = sqrt(pow(yCurrent - yt, 2.0) + pow(xCurrent - xt, 2.0));
    if (distance < distanceMin)
    {
      distanceMin = distance;

      freneState.ts = tst;
      freneState.s = st;
      freneState.dts = dtst;
      freneState.ddts = ddtst;
      freneState.dddts = dddtst;
      freneState.td = tdt;
      freneState.d = dt;
      freneState.dtd = dtdt;
      freneState.ddtd = ddtdt;
      freneState.dddtd = dddtdt;
    }
  }
}

void Frene::filterByOptimalityCostFromCartesianTrajectoryArray(
    std::vector<std::vector<std::vector<double> > >& trajectoryArray,
    std::vector<std::vector<std::vector<double> > >& optimalTrajectoryArray, double kjlong, double ktlong,
    double kplong, double kjlat, double ktlat, double kplat, double klong, double klat, int freneMode,
    double desiredSpeed, double desiredLatPosition,  // for velocity keeping
    double desiredLongPosition                       // for following, stopping
    )
{
  double totalCostMin = 1e+14;
  std::vector<std::vector<std::vector<double> > >::iterator itrOptimal;  // 2d matrix

  for (std::vector<std::vector<std::vector<double> > >::iterator itr0 = trajectoryArray.begin();
       itr0 != trajectoryArray.end(); itr0++)
  {
    // itr0->2d matrix, itr1->1d vector
    // compute cost at the trajectory end point
    std::vector<double> itr1 = itr0->back();

    FreneState freneStateEnd;
    if (itr1.size() < FreneState::size())
    {
      ROS_ERROR("Merged trajectory size is incorrect");
      return;
    }

    freneStateEnd.ts = itr1[8];
    freneStateEnd.s = itr1[9];
    freneStateEnd.dts = itr1[10];
    freneStateEnd.ddts = itr1[11];
    freneStateEnd.dddts = itr1[12];
    freneStateEnd.sumdddts = itr1[13];

    freneStateEnd.td = itr1[14];
    freneStateEnd.d = itr1[15];
    freneStateEnd.dtd = itr1[16];
    freneStateEnd.ddtd = itr1[17];
    freneStateEnd.dddtd = itr1[18];
    freneStateEnd.sumdddtd = itr1[19];

    double costLong = 0;
    switch (freneMode)
    {
      case 1:
      case 2:  // following, stopping
        costLong = kjlong * freneStateEnd.sumdddts + ktlong * freneStateEnd.ts +
                   kplong * fabs(freneStateEnd.s - desiredLongPosition);
        break;
      default:  // 0, velocity keeping
        costLong = kjlong * freneStateEnd.sumdddts + ktlong * freneStateEnd.ts +
                   kplong * fabs(freneStateEnd.dts - desiredSpeed);
        break;
    }
    double costLat =
        kjlat * freneStateEnd.sumdddtd + ktlat * freneStateEnd.td + kplat * fabs(freneStateEnd.d - desiredLatPosition);
    double totalCost = klong * costLong + klat * costLat;

    if (totalCost < totalCostMin)
    {
      totalCostMin = totalCost;
      itrOptimal = itr0;
    }
  }

  optimalTrajectoryArray.clear();
  optimalTrajectoryArray.push_back(*itrOptimal);
}

void Frene::computeOptimalTrajectory(std::vector<std::vector<std::vector<double> > >& trajectoryArray,
                                     std::vector<std::vector<std::vector<double> > >& paramArray,
                                     std::vector<std::vector<std::vector<double> > >& optimalTrajectoryArray,
                                     std::vector<std::vector<std::vector<double> > >& optimalParamArray, double kjlong,
                                     double ktlong, double kplong, double kjlat, double ktlat, double kplat,
                                     double klong, double klat, double desiredSpeed, double desiredLatPosition)
{
  double totalCostMin = 1e+14;
  std::vector<std::vector<std::vector<double> > >::iterator itrOptimal;       // 2d matrix
  std::vector<std::vector<std::vector<double> > >::iterator itrOptimalParam;  // 2d matrix

  // iterator for param
  std::vector<std::vector<std::vector<double> > >::iterator itrParam = paramArray.begin();

  for (std::vector<std::vector<std::vector<double> > >::iterator itr0 = trajectoryArray.begin();
       itr0 != trajectoryArray.end(); itr0++)
  {
    // itr0->2d matrix, itr1->1d vector
    // compute cost at the trajectory end point
    std::vector<double> itr1 = itr0->back();

    FreneState freneStateEnd;
    if (itr1.size() < FreneState::size())
    {
      ROS_ERROR("Merged trajectory size is incorrect");
      return;
    }
    freneStateEnd.ts = itr1[0];
    freneStateEnd.s = itr1[1];
    freneStateEnd.dts = itr1[2];
    freneStateEnd.ddts = itr1[3];
    freneStateEnd.dddts = itr1[4];
    freneStateEnd.sumdddts = itr1[5];

    freneStateEnd.td = itr1[6];
    freneStateEnd.d = itr1[7];
    freneStateEnd.dtd = itr1[8];
    freneStateEnd.ddtd = itr1[9];
    freneStateEnd.dddtd = itr1[10];
    freneStateEnd.sumdddtd = itr1[11];

    double costLong =
        kjlong * freneStateEnd.sumdddts + ktlong * freneStateEnd.ts + kplong * fabs(freneStateEnd.dts - desiredSpeed);
    double costLat =
        kjlat * freneStateEnd.sumdddtd + ktlat * freneStateEnd.td + kplat * fabs(freneStateEnd.d - desiredLatPosition);
    double totalCost = klong * costLong + klat * costLat;

    if (totalCost < totalCostMin)
    {
      totalCostMin = totalCost;
      itrOptimal = itr0;
      itrOptimalParam = itrParam;
    }

    itrParam++;
  }

  optimalTrajectoryArray.clear();
  optimalTrajectoryArray.push_back(*itrOptimal);

  optimalParamArray.clear();
  optimalParamArray.push_back(*itrOptimalParam);
}

void Frene::computeTrajectoryZero(std::vector<std::vector<std::vector<double> > >& previousOptimalTrajectory,
                                  std::vector<std::vector<std::vector<double> > >& trajectoryZero,
                                  double trajectoryLength, int numOfPoints)
{
  trajectoryZero.clear();

  if (previousOptimalTrajectory.size() <= 0)
  {
    return;
  }
  double x0 = previousOptimalTrajectory[0][0][0];
  double y0 = previousOptimalTrajectory[0][0][1];
  double theta0 = previousOptimalTrajectory[0][0][3];  // yaw angle

  Eigen::VectorXd p0(2);
  p0 << x0, y0;
  Eigen::VectorXd d0(2);
  d0 << cos(theta0), sin(theta0);

  Eigen::VectorXd p(2);
  double distance = 0;
  double delta = trajectoryLength / (double)(numOfPoints - 1);

  std::vector<double> point(21);
  std::vector<std::vector<double> > tmp;
  tmp.clear();

  for (int itr = 0; itr < numOfPoints; itr++)
  {
    distance = delta * itr;
    p = p0 + distance * d0;

    point[0] = p(0);    // x
    point[1] = p(1);    // y
    point[2] = itr;     // t
    point[3] = theta0;  // theta
    point[4] = 0;       // dtTheta
    point[5] = 0;       // vx
    point[6] = 0;       // ax
    point[7] = 0;       // kx
    point[8] = itr;     // ts
    point[9] = 0;       // s
    point[10] = 0;      // dts
    point[11] = 0;      // ddts
    point[12] = 0;      // dddts
    point[13] = 0;      // sumdddts
    point[14] = itr;    // td
    point[15] = 0;      // d
    point[16] = 0;      // dtd
    point[17] = 0;      // ddtd
    point[18] = 0;      // dddtd
    point[19] = 0;      // sumdddtd
    point[20] = 0;      // centrifugalForce

    tmp.push_back(point);
  }

  trajectoryZero.push_back(tmp);
}

void Frene::computeTrajectorySmoothStopping(std::vector<std::vector<std::vector<double> > >& previousOptimalTrajectory,
                                            std::vector<std::vector<std::vector<double> > >& trajectoryZero,
                                            double trajectoryLength, double smoothnessFactor, int numOfPoints)
{
  trajectoryZero.clear();

  if (previousOptimalTrajectory.size() <= 0)
  {
    return;
  }
  double x0 = previousOptimalTrajectory[0][0][0];
  double y0 = previousOptimalTrajectory[0][0][1];
  double theta0 = previousOptimalTrajectory[0][0][3];  // yaw angle
  double vx0 = previousOptimalTrajectory[0][0][5];

  Eigen::VectorXd p0(2);
  p0 << x0, y0;
  Eigen::VectorXd d0(2);
  d0 << cos(theta0), sin(theta0);

  Eigen::VectorXd p(2);
  double distance = 0;
  double distanceNext = 0;
  double delta = trajectoryLength / (double)(numOfPoints - 1);
  double vx = 0;

  std::vector<double> point(21);
  std::vector<std::vector<double> > tmp;
  tmp.clear();

  for (int itr = 0; itr < numOfPoints; itr++)
  {
    distance = delta * itr;
    distanceNext = delta * (itr + 1);
    p = p0 + distance * d0;
    vx = vx0 * exp(-smoothnessFactor * distanceNext);

    point[0] = p(0);    // x
    point[1] = p(1);    // y
    point[2] = itr;     // t
    point[3] = theta0;  // theta
    point[4] = 0;       // dtTheta
    point[5] = vx;      // vx
    point[6] = 0;       // ax
    point[7] = 0;       // kx
    point[8] = itr;     // ts
    point[9] = 0;       // s
    point[10] = 0;      // dts
    point[11] = 0;      // ddts
    point[12] = 0;      // dddts
    point[13] = 0;      // sumdddts
    point[14] = itr;    // td
    point[15] = 0;      // d
    point[16] = 0;      // dtd
    point[17] = 0;      // ddtd
    point[18] = 0;      // dddtd
    point[19] = 0;      // sumdddtd
    point[20] = 0;      // centrifugalForce

    tmp.push_back(point);
  }

  trajectoryZero.push_back(tmp);
}

void Frene::filterBySafety(std::vector<std::vector<std::vector<double> > >& trajectoryArrayIn,
                           std::vector<std::vector<std::vector<double> > >& trajectoryArrayOut, double roadMarginRight,
                           double roadMarginLeft, double maxCentrifugalForce, double maxAcceleration,
                           double maxCurvature)
{
  trajectoryArrayOut.clear();
  double minValue = 1e+14;  // to find second best solution
  std::vector<std::vector<double> >::iterator itrMin;

  for (std::vector<std::vector<std::vector<double> > >::iterator itr1 = trajectoryArrayIn.begin();
       itr1 != trajectoryArrayIn.end(); itr1++)
  {
    std::vector<std::vector<double> > tmp;
    bool isSafe = true;

    for (std::vector<std::vector<double> >::iterator itr2 = itr1->begin(); itr2 != itr1->end(); itr2++)
    {
      double x = (*itr2)[0];
      double y = (*itr2)[1];
      double t = (*itr2)[2];
      double theta = (*itr2)[3];
      double dtTheta = (*itr2)[4];
      double vx = (*itr2)[5];
      double ax = (*itr2)[6];
      double kx = (*itr2)[7];

      double ts = (*itr2)[8];
      double s = (*itr2)[9];
      double dts = (*itr2)[10];
      double ddts = (*itr2)[11];
      double dddts = (*itr2)[12];
      double sumdddts = (*itr2)[13];

      double td = (*itr2)[14];
      double d = (*itr2)[15];
      double dtd = (*itr2)[16];
      double ddtd = (*itr2)[17];
      double dddtd = (*itr2)[18];
      double sumdddtd = (*itr2)[19];

      double centrifugalForce = (*itr2)[20];

      if (ax < minValue)
      {
        minValue = ax;
        itrMin = itr2;
      }

      if (fabs(centrifugalForce) > maxCentrifugalForce)
      {
        isSafe = false;
        break;
      }
      if (ax > maxAcceleration)
      {
        isSafe = false;
        break;
      }
      if (fabs(kx) > maxCurvature)
      {
        isSafe = false;
        break;
      }
      if (d > roadMarginLeft || d < -roadMarginRight)
      {
        // d is positive in left direction
        isSafe = false;
        break;
      }

      tmp.push_back(*itr2);
    }

    if (isSafe)
    {
      trajectoryArrayOut.push_back(tmp);
    }
    else
    {
      // filter out
    }
  }
}

void Frene::setAsInvalidTrajectory(std::vector<std::vector<std::vector<double> > >& path)
{
  if (path.size() <= 0)
  {
    ROS_ERROR("Frene::path size is invalid.");
    return;
  }
  if (path[0].size() <= 0)
  {
    ROS_ERROR("Frene::path size is invalid.");
    return;
  }
  if (path[0][0].size() <= 2)
  {
    ROS_ERROR("Frene::path size is invalid.");
    return;
  }

  // set negative time value to the first trajectory element
  path[0][0][2] = -1;
}

bool Frene::isInvalidTrajectory(std::vector<std::vector<std::vector<double> > >& path)
{
  if (path.size() <= 0)
  {
    return true;
  }
  if (path[0].size() <= 0)
  {
    return true;
  }
  if (path[0][0].size() <= 2)
  {
    return true;
  }

  // set negative time value to the first trajectory element
  bool isIllegal = false;

  if (path[0][0][2] < 0)
  {
    isIllegal = true;
  }

  return isIllegal;
}

double Frene::getGroundMapZ()
{
  static bool init = false;
  static double map_z = 0;

  if (!init)
  {
    geometry_msgs::PointStamped world_0, map_0;
    world_0.header.frame_id = "/world";
    world_0.point.x = 0;
    world_0.point.y = 0;
    world_0.point.z = 0;
    tf::TransformListener tf;
    try{
      tf.waitForTransform("/map", "/world", ros::Time(0), ros::Duration(1.0));
    }catch(tf::TransformException &ex){
      ROS_ERROR("%s", ex.what());
    }
    tf.transformPoint("/map", ros::Time(0), world_0, "/world", map_0);
    map_z = map_0.point.z;
    init = true;
  }
  return map_z;
}

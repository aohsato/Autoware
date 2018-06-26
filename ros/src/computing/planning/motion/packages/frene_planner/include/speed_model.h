#ifndef SPEEDMODEL_H
#define SPEEDMODEL_H

#include <vector>
#include <string>
#include "wall_manager.h"

// classes and structures
class SpeedModel
{
private:
public:
  SpeedModel(void);
  void filterByInvisibleDanger(std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListIn,
                               std::vector<std::vector<std::vector<double> > >& trajectoryArrayCartesianListOut,
                               std::vector<BlindCorner>& blindCornerList, double x0, double y0, double yaw,
                               double paramSpeedModelVo, double paramSpeedModelDy, double paramSpeedModelAd,
                               double paramSpeedModelAmax, double paramSpeedModelDv);
  double computeSafeSpeed(double xein, double yein, double vxe, double vye, bool isLeft, double vo, double dy,
                          double ad, double amax, double dv, double& ve0out, double& y0out);

  void computeSafeSpeedForBlindCorner(BlindCorner& blindCorner, std::vector<std::vector<std::vector<double> > >& result,
                                      double vo, double dy, double ad, double amax, double dv);
};

#endif

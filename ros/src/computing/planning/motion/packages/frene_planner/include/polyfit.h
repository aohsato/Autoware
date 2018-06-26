#ifndef POLYFIT_H
#define POLYFIT_H
#include <vector>

class PolyFit
{
private:
  int _polyOrder;
  std::vector<double> _cc;

public:
  PolyFit(void){};
  void init(int polyOrder);
  void setPoints(std::vector<double>& x, std::vector<double>& y);
  double value(double x0);
  double deriv(int order, double x0);
};

#endif

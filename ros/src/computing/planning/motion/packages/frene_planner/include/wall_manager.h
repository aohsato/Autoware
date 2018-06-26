#ifndef WALLMANAGER_H
#define WALLMANAGER_H

#include <vector>
#include <string>

// classes and structures
class CornerNode
{
private:
public:
  int id;
  double x;
  double y;
  double z;

  CornerNode(void)
  {
    id = 0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
  };

  CornerNode& operator=(const CornerNode& rhs)
  {
    id = rhs.id;
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;

    return *this;
  }
};

class WallNode
{
private:
public:
  std::vector<CornerNode> corner;
  int id;
  int numCorner;

  WallNode(void){};

  void resizeCorner(int numCorner)
  {
    corner.clear();
    corner.resize(numCorner);
  };
};

class BlindCorner
{
private:
public:
  double x;
  double y;
  double z;
  double vx;
  double vy;
  double vz;
  bool isLeft;

  BlindCorner(void)
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    vx = 0.0;
    vy = 0.0;
    vz = 0.0;
    isLeft = true;
  };

  BlindCorner& operator=(const BlindCorner& rhs)
  {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
    vx = rhs.vx;
    vy = rhs.vy;
    vz = rhs.vz;
    isLeft = rhs.isLeft;

    return *this;
  }
};

class WallManager
{
private:
public:
  WallManager(void);

  int createWallNodeListFromTxt(std::string& filename, std::vector<WallNode>& result);
  void findForwardBlindCorner(std::vector<WallNode>& wallNodeList, double x0, double y0, double yaw,
                              std::vector<BlindCorner>& result);
};

#endif

#ifndef BOX_H
#define BOX_H
#include "stdafx.h"


class Box : public Body
{
public:
  Box();
  ~Box();
  float* GetPoints();
  float* GetOBB();

private:
  float pts[24];

};

#endif // BOX_H

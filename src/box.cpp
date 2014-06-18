#include "stdafx.h"

//GLuint Box::indices[24] = {
//      0, 1, 2, 3,                 // Front face
//      7, 4, 5, 6,                 // Back face
//      6, 5, 2, 1,                 // Left face
//      7, 0, 3, 4,                 // Right face
//      7, 6, 1, 0,                 // Top face
//      3, 2, 5, 4,                 // Bottom face
//  };

Box::Box()
{
}

Box::~Box()
{

}

float* Box::GetPoints()
{
  float* center = GetCenter();
  float* lengths = GetLengths();
  pts[0] = center[0]+lengths[0];
  pts[1] = center[1]+lengths[1];
  pts[2] = center[2]+lengths[2];
  pts[3] = center[0]-lengths[0];
  pts[4] = center[1]+lengths[1];
  pts[5] = center[2]+lengths[2];
  pts[6] = center[0]-lengths[0];
  pts[7] = center[1]-lengths[1];
  pts[8] = center[2]+lengths[2];
  pts[9] = center[0]+lengths[0];
  pts[10] = center[1]-lengths[1];
  pts[11] = center[2]+lengths[2];
  pts[12] = center[0]+lengths[0];
  pts[13] = center[1]-lengths[1];
  pts[14] = center[2]-lengths[2];
  pts[15] = center[0]-lengths[0];
  pts[16] = center[1]-lengths[1];
  pts[17] = center[2]-lengths[2];
  pts[18] = center[0]-lengths[0];
  pts[19] = center[1]+lengths[1];
  pts[20] = center[2]-lengths[2];
  pts[21] = center[0]+lengths[0];
  pts[22] = center[1]+lengths[1];
  pts[23] = center[2]-lengths[2];
  return pts;
}

float* Box::GetOBB()
{
  GetPoints();
  float* center = GetCenter();
  for(int i=0;i<8;i++)
  {
      float p[3] = {pts[i*3]-center[0],pts[i*3+1]-center[1], pts[i*3+2]-center[2]};
      QuaternionTransform(p, GetOrientation());
      pts[i*3] = p[0]+center[0];
      pts[i*3+1] = p[1]+center[1];
      pts[i*3+2] = p[2]+center[2];
  }

  return pts;
}

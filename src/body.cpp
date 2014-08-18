#include "stdafx.h"

Body::Body()
{
  weight = 1;
  SetLengths(1,1,1);
  SetVelocity(0,0,0);
  angularForce[0] = 0.0f;
  angularForce[1] = 0.0f;
  angularForce[2] = 0.0f;
}

Body::~Body()
{

}

float* Body::GetCenter()
{
  return center;
}

void Body::SetCenter(float x, float y, float z)
{
  center[0] = x;
  center[1] = y;
  center[2] = z;
}

float* Body::GetVelocity()
{
  return velocity;
}

void Body::SetVelocity(float vx, float vy, float vz)
{
  velocity[0] = vx;
  velocity[1] = vy;
  velocity[2] = vz;
}

void Body::AddForce(float x, float y, float z)
{
  velocity[0] += x;
  velocity[1] += y;
  velocity[2] += z;
}

void Body::AddAngularForce(float x, float y, float z)
{
  angularForce[0] += AngleToRad(x);
  angularForce[1] += AngleToRad(y);
  angularForce[2] += AngleToRad(z);
}

float* Body::GetAngularForce()
{
  return angularForce;
}

float* Body::GetOrientation()
{
  return orientation;
}

void Body::SetOrientation(float ox, float oy, float oz, float ow)
{
  orientation[0] = ox;
  orientation[1] = oy;
  orientation[2] = oz;
  orientation[3] = ow;
}

float* Body::GetAngularVelocity()
{
  return angularVelocity;
}

void Body::SetAngularVelocity(float vx, float vy, float vz)
{
  angularVelocity[0] = vx;
  angularVelocity[1] = vy;
  angularVelocity[2] = vz;
  CalculateOrientation();
}

float* Body::GetAngularVelocityAsDegree()
{float* deg = new float[3];
  deg[0] = angularVelocity[0] / ONE_DEG_IN_RAD;
  deg[1] = angularVelocity[1] / ONE_DEG_IN_RAD;
  deg[2] = angularVelocity[2] / ONE_DEG_IN_RAD;
  return deg;
}

float* Body::GetLengths()
{
  return lengths;
}

void Body::SetLengths(float x, float y, float z)
{
  lengths[0] = x;
  lengths[1] = y;
  lengths[2] = z;
}

float Body::GetWeight()
{
  return weight;
}

void Body::SetWeight(float w)
{
  weight = w;
}

float* Body::GetColor()
{
  return color;
}

void Body::CalculateOrientation()
{
  float cos_z_2 = cosf(0.5*angularVelocity[2]);
  float cos_y_2 = cosf(0.5*angularVelocity[1]);
  float cos_x_2 = cosf(0.5*angularVelocity[0]);

  float sin_z_2 = sinf(0.5*angularVelocity[2]);
  float sin_y_2 = sinf(0.5*angularVelocity[1]);
  float sin_x_2 = sinf(0.5*angularVelocity[0]);

  orientation[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
  orientation[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
  orientation[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
  orientation[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;
}

float* Body::QuaternionTransform(float* point, float* q)
{
  float xx = q[1] * q[0], yy = q[2] * q[2], zz = q[3] * q[3],
      xy = q[1] * q[2], xz = q[1] * q[3],
      yz = q[2] * q[3], wx = q[0] * q[1],
      wy = q[0] * q[2], wz = q[0] * q[3];
  float tmp[3] ={(1.0f - 2.0f * ( yy + zz )) * point[0] + (2.0f * ( xy - wz )) * point[1] + (2.0f * ( xz + wy )) * point[2],
                 (2.0f * ( xy + wz )) * point[0] + (1.0f - 2.0f * ( xx + zz )) * point[1] + (2.0f * ( yz - wx )) * point[2],
                 (2.0f * ( xz - wy )) * point[0] + (2.0f * ( yz + wx )) * point[1] + (1.0f - 2.0f * ( xx + yy )) * point[2]};
  point[0] = tmp[0];
  point[1] = tmp[1];
  point[2] = tmp[2];
  return point;
}

int Body::draw()
{
  return -1;
}

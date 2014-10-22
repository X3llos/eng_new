#ifndef BODY_H
#define BODY_H
#include "stdafx.h"

enum BodyType
{
  tBox,
  tSphere,
  tCapsule
};

static inline float AngleToRad(float deg)
{
  return ONE_DEG_IN_RAD*deg;
}

static inline float RadToAngle(float deg)
{
  return deg/ONE_DEG_IN_RAD;
}

class Body
{
public:
  Body();
  virtual ~Body();
  float* GetCenter();
  void SetCenter(float, float, float);
  float* GetVelocity();
  void SetVelocity(float, float, float);
  void AddForce(float, float, float);
  void AddAngularForce(float, float, float);
  float* GetAngularForce();
  float* GetOrientation();
  void SetOrientation(float, float, float, float);
  float* GetAngularVelocity();
  void SetAngularVelocity(float, float, float);
  float* GetAngularVelocityAsDegree();
  float* GetLengths();
  void SetLengths(float, float, float);
  float GetWeight();
  void SetWeight(float);

  void CalculateOrientation();
  float* QuaternionTransform(float* Point, float* quat);

  virtual int draw();
  virtual float* GetPoints(){return NULL;};
  virtual float* GetOBB(){return NULL;};
  float* GetColor();

  bool isActive;
  BodyType type;

private:
  float center[3];
  float lengths[3];
  float velocity[3];
  float angularForce[3];
  float angularVelocity[3];
  float weight;
  float color[3];
  float orientation[4]; //w, x, y, z
};
#endif // BODY_H

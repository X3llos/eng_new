#ifndef BODYINST_H
#define BODYINST_H
#include "stdafx.h"

//enum BodyType
//{
//  tBox,
//  tSphere,
//  tCapsule
//};

//static inline float AngleToRad(float deg)
//{
//  return ONE_DEG_IN_RAD*deg;
//}

//static inline float RadToAngle(float deg)
//{
//  return deg/ONE_DEG_IN_RAD;
//}
struct myBody{
bool isActive;
BodyType type;
float center[3];
float lengths[3];
float velocity[3];
float angularForce[3];
float angularVelocity[3];
float weight;
float color[3];
float orientation[4]; //w, x, y, z
float pts[24];
};
class BodyInst
{
public:
  BodyInst();
  static BodyInst& GetInstance()
  {
    static BodyInst instance;
    return instance;
  }
  virtual ~BodyInst();
  void SetCenter(float, float, float, myBody);
  void SetVelocity(float, float, float, myBody);
  void AddForce(float, float, float, myBody);
  void AddAngularForce(float, float, float, myBody);
  void SetOrientation(float, float, float, float, myBody);
  void SetAngularVelocity(float, float, float, myBody);
  float* GetAngularVelocityAsDegree(myBody);
  void SetLengths(float, float, float, myBody);
  void SetWeight(float, myBody);

  void GetPoints(myBody);
  void GetOBB(myBody);
  void CalculateOrientation(myBody);
  float* QuaternionTransform(float* Point, float* quat);

  virtual int draw();
  float* GetColor();
};
#endif // BODYINST_H

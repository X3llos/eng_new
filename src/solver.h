#ifndef SOLVER_H
#define SOLVER_H
#include "stdafx.h"
class Solver
{
public:

  ~Solver();

  int Init();

  //helpers
//  cl_float4 QuaternionMul(cl_float4 q1, cl_float4 q2);
//  glm::vec4 QuaternionMul(glm::vec4 q1, glm::vec4 q2);
//  cl_float3 QuaternionTransform(cl_float3 p,  cl_float4 q);
//  glm::vec3 QuaternionTransform(glm::vec3 p,  glm::vec4 q);
//  float getDistance(cl_float3 v1, cl_float3 v2);
//  float getDistance(glm::vec3 v1, glm::vec3 v2);
//  cl_float3 Radians(cl_float3 d);
//  glm::vec3 Radians(glm::vec3 d);
//  float GetLength(cl_float3 vec);
//  float GetLength(glm::vec3 vec);
//  float GetLength4(cl_float4 vec);
//  cl_float4 Normalize(cl_float4);
  static inline void QuatToMat(float* q, float *&mat)
  {
    mat[0] = 1 - 2*q[2]*q[2] - 2*q[3]*q[3];
    mat[1] = 2*q[1]*q[2] + 2*q[0]*q[3];
    mat[2] = 2*q[1]*q[3] - 2*q[0]*q[2];
    mat[3] = 2*q[1]*q[2] - 2*q[0]*q[3];
    mat[4] = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
    mat[5] = 2*q[2]*q[3] + 2*q[0]*q[1];
    mat[6] = 2*q[1]*q[3] + 2*q[0]*q[2];
    mat[7] = 2*q[2]*q[3] - 2*q[0]*q[1];
    mat[8] =  1 - 2*q[1]*q[1] - 2*q[2]*q[2];
  }

  static inline void MultMatByVec(float *&vec, float* mat)
  {
    float v[3];
    v[0] = vec[0]* mat[0] + vec[1]*mat[1] + vec[2]*mat[2];
    v[1] = vec[0]* mat[3] + vec[1]*mat[4] + vec[2]*mat[5];
    v[2] = vec[0]* mat[6] + vec[1]*mat[7] + vec[2]*mat[8];
    vec[0] = v[0];
    vec[1] = v[1];
    vec[2] = v[2];
  }

  static inline float CalcDot(float *v1, float *v2)
  {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
  }

  static inline void CalcCross(float *v1, float *v2, float *v3)
  {
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
  }

  bool TestAxisSAT(float* ptsA,float* ptsB, float* axis, float &collLen, int &collAxis);

  //Updates
  void UpdateVelocity(Body* obj, double &timeStep);
  void UpdateAngularVelocity(Body* obj, double &timeStep);
  int Update(double , Body* points, int, int first);
  int UpdateCPU(double,std::vector<Body*>, int first);

  bool CheckCollision(Body* a, Body* b, float &collisionLen, int &collisionAxis);

  int Close();

  static Solver& GetInstance()
  {
    static Solver instance;
    return instance;
  }

private:
  Solver();
  Solver(const Solver&);
  cl_device_id device_id;
  cl_context context;
  cl_command_queue command_queue;
  cl_program program;
  cl_kernel kernel;
  cl_platform_id platform_id;
  cl_uint ret_num_devices;
  cl_uint ret_num_platforms;
  cl_int ret;

  FILE *fp;
  char *fileName;
  char *source_str;
  size_t source_size;
  cl_mem cl_a;
  //data;

  bool wasText;
};

#endif // SOLVER_H

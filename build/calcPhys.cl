#pragma OPENCL EXTENSION cl_khr_fp64 : enable
//enum BodyType
//{
//  tBox,
//  tSphere,
//  tCapsule
//};
typedef struct
{
    bool isActive;
//    BodyType type;
    float center[3];
    float lengths[3];
    float velocity[3];
    float angularForce[3];
    float angularVelocity[3];
    float weight;
    float color[3];
    float orientation[4]; //w, x, y, z
    float pts[24];
} myBody;

//float AngleToRad(float deg)
//{
//  return M_PI/180*deg;
//}

//void SetCenter(float x, float y, float z, myBody *b)
//{
//  b->center[0] = x;
//  b->center[1] = y;
//  b->center[2] = z;
//}

//void SetVelocity(float vx, float vy, float vz, myBody *b)
//{
//  b->velocity[0] = vx;
//  b->velocity[1] = vy;
//  b->velocity[2] = vz;
//}

//void AddForce(float x, float y, float z, myBody *b)
//{
//  b->velocity[0] += x;
//  b->velocity[1] += y;
//  b->velocity[2] += z;
//}

//void AddAngularForce(float x, float y, float z, myBody *b)
//{
//  b->angularForce[0] += AngleToRad(x);
//  b->angularForce[1] += AngleToRad(y);
//  b->angularForce[2] += AngleToRad(z);
//}

//void SetOrientation(float ox, float oy, float oz, float ow, myBody *b)
//{
//  b->orientation[0] = ox;
//  b->orientation[1] = oy;
//  b->orientation[2] = oz;
//  b->orientation[3] = ow;
//}

//void SetAngularVelocity(float vx, float vy, float vz, myBody *b)
//{
//  b->angularVelocity[0] = vx;
//  b->angularVelocity[1] = vy;
//  b->angularVelocity[2] = vz;
//  CalculateOrientation(b);
//}

//float* GetAngularVelocityAsDegree(myBody *b)
//{
//  float* deg = new float[3];
//  deg[0] = b->angularVelocity[0] / (M_PI/180.0);
//  deg[1] = b->angularVelocity[1] / (M_PI/180.0);
//  deg[2] = b->angularVelocity[2] / (M_PI/180.0);
//  return deg;
//}

//void SetLengths(float x, float y, float z, myBody *b)
//{
//  b->lengths[0] = x;
//  b->lengths[1] = y;
//  b->lengths[2] = z;
//}

//void SetWeight(float w, myBody *b)
//{
//  b->weight = w;
//}

//void CalculateOrientation(myBody *b)
//{
//  float cos_z_2 = cosf(0.5*b->angularVelocity[2]);
//  float cos_y_2 = cosf(0.5*b->angularVelocity[1]);
//  float cos_x_2 = cosf(0.5*b->angularVelocity[0]);

//  float sin_z_2 = sinf(0.5*b->angularVelocity[2]);
//  float sin_y_2 = sinf(0.5*b->angularVelocity[1]);
//  float sin_x_2 = sinf(0.5*b->angularVelocity[0]);

//  b->orientation[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
//  b->orientation[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
//  b->orientation[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
//  b->orientation[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;
//}

//float* QuaternionTransform(float* point, float* q)
//{
//  float xx = q[1] * q[0], yy = q[2] * q[2], zz = q[3] * q[3],
//      xy = q[1] * q[2], xz = q[1] * q[3],
//      yz = q[2] * q[3], wx = q[0] * q[1],
//      wy = q[0] * q[2], wz = q[0] * q[3];
//  float tmp[3] ={(1.0f - 2.0f * ( yy + zz )) * point[0] + (2.0f * ( xy - wz )) * point[1] + (2.0f * ( xz + wy )) * point[2],
//                 (2.0f * ( xy + wz )) * point[0] + (1.0f - 2.0f * ( xx + zz )) * point[1] + (2.0f * ( yz - wx )) * point[2],
//                 (2.0f * ( xz - wy )) * point[0] + (2.0f * ( yz + wx )) * point[1] + (1.0f - 2.0f * ( xx + yy )) * point[2]};
//  point[0] = tmp[0];
//  point[1] = tmp[1];
//  point[2] = tmp[2];
//  return point;
//}

//void GetPoints(myBody *b)
//{
//  b->pts[0] = b->center[0]+b->lengths[0];
//  b->pts[1] = b->center[1]+b->lengths[1];
//  b->pts[2] = b->center[2]+b->lengths[2];
//  b->pts[3] = b->center[0]-b->lengths[0];
//  b->pts[4] = b->center[1]+b->lengths[1];
//  b->pts[5] = b->center[2]+b->lengths[2];
//  b->pts[6] = b->center[0]-b->lengths[0];
//  b->pts[7] = b->center[1]-b->lengths[1];
//  b->pts[8] = b->center[2]+b->lengths[2];
//  b->pts[9] = b->center[0]+b->lengths[0];
//  b->pts[10] = b->center[1]-b->lengths[1];
//  b->pts[11] = b->center[2]+b->lengths[2];
//  b->pts[12] = b->center[0]+b->lengths[0];
//  b->pts[13] = b->center[1]-b->lengths[1];
//  b->pts[14] = b->center[2]-b->lengths[2];
//  b->pts[15] = b->center[0]-b->lengths[0];
//  b->pts[16] = b->center[1]-b->lengths[1];
//  b->pts[17] = b->center[2]-b->lengths[2];
//  b->pts[18] = b->center[0]-b->lengths[0];
//  b->pts[19] = b->center[1]+b->lengths[1];
//  b->pts[20] = b->center[2]-b->lengths[2];
//  b->pts[21] = b->center[0]+b->lengths[0];
//  b->pts[22] = b->center[1]+b->lengths[1];
//  b->pts[23] = b->center[2]-b->lengths[2];
//}

//void GetOBB(myBody *b)
//{
//  GetPoints(b);
//  for(int i=0;i<8;i++)
//  {
//      float p[3] = {b->pts[i*3]-b->center[0],b->pts[i*3+1]-b->center[1], b->pts[i*3+2]-b->center[2]};
//      QuaternionTransform(p, b->orientation);
//      b->pts[i*3] = p[0]+b->center[0];
//      b->pts[i*3+1] = p[1]+b->center[1];
//      b->pts[i*3+2] = p[2]+b->center[2];
//  }
//}

//static inline float CalcDot(float *v1, float *v2)
//{
//  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
//}

//bool TestAxisSAT(float* ptsA, float* ptsB, float* axis, float &collPoint)
//{
//  float* tmpPtA = new float[3]();
//  float* tmpPtB = new float[3]();
//  float minval1 = FLT_MAX;
//  float maxval1 = -FLT_MAX;
//  float minval2 = FLT_MAX;
//  float maxval2 = -FLT_MAX;
//  for( int i = 0 ; i < 8 ; i++ )
//  {
//    tmpPtA[0] = ptsA[i*3];
//    tmpPtA[1] = ptsA[i*3+1];
//    tmpPtA[2] = ptsA[i*3+2];
//    tmpPtB[0] = ptsB[i*3];
//    tmpPtB[1] = ptsB[i*3+1];
//    tmpPtB[2] = ptsB[i*3+2];
//    float dotVal = CalcDot(tmpPtA, axis);
//    if( dotVal < minval1 )  minval1=dotVal;
//    if( dotVal > maxval1 )  maxval1=dotVal;
//    dotVal = CalcDot(tmpPtB, axis);
//    if( dotVal < minval2 )  minval2=dotVal;
//    if( dotVal > maxval2 )  maxval2=dotVal;
//  }
//  delete tmpPtA;
//  delete tmpPtB;
//  if(!((minval2 >= minval1 && minval2 <= maxval1) || (minval1 >= minval2 && minval1 <= maxval2)))
//  {
//    return false;
//  }
//  collPoint = maxval1 + minval1;
//  return true;
//}

//float* PenetrationDepthCorrection(float* ptsA, float* ptsB, float* axis,
//                                        float &collLen, int &collAxis, int actAxis, float *collPoint)
//{
//  float* tmpPtA = new float[3]();
//  float* tmpPtB = new float[3]();
//  float* tmppt2 = new float[3]();
//  float minval1 = FLT_MAX;
//  float maxval1 = -FLT_MAX;
//  float minval2 = FLT_MAX;
//  float maxval2 = -FLT_MAX;
//  tmppt2[0] = FLT_MAX;

//  float tmpcollLen;
//  for( int i = 0 ; i < 8 ; i++ )
//  {
//    tmpPtA[0] = ptsA[i*3];
//    tmpPtA[1] = ptsA[i*3+1];
//    tmpPtA[2] = ptsA[i*3+2];
//    tmpPtB[0] = ptsB[i*3];
//    tmpPtB[1] = ptsB[i*3+1];
//    tmpPtB[2] = ptsB[i*3+2];

//    float dotVal = CalcDot(tmpPtA, axis);
//    if( dotVal < minval1 )  minval1=dotVal;
//    if( dotVal > maxval1 )  maxval1=dotVal;
//    dotVal = CalcDot(tmpPtB, axis);
//    if( dotVal < minval2 )  minval2=dotVal;
//    if( dotVal > maxval2 )  maxval2=dotVal;

//    if (std::abs(tmpPtA[0]) == std::abs(collLen) || std::abs(tmpPtA[1]) == std::abs(collLen) || std::abs(tmpPtA[2]) == std::abs(collLen))
//    {
//      if (tmppt2[0] == FLT_MAX)
//      {
//        tmppt2[0] = tmpPtA[0];
//        tmppt2[1] = tmpPtA[1];
//        tmppt2[2] = tmpPtA[2];
//      }
//      else
//      {
//        tmppt[0] = tmpPtA[0];
//        tmppt[1] = tmpPtA[1];
//        tmppt[2] = tmpPtA[2];
//      }
//    }
//  }
//  tmpcollLen = maxval1 > maxval2 ? (maxval2 - minval1) : (maxval1 - minval2);
//  if(abs(collLen) > abs(tmpcollLen))
//  {
//    collLen = tmpcollLen;
//    collAxis = actAxis;
//  }
//  tmppt[0] = tmppt2[0] != tmppt[0] ? (tmppt2[0] + tmppt[0])/2.0 : tmppt[0];
//  tmppt[1] = tmppt2[1] != tmppt[1] ? (tmppt2[1] + tmppt[1])/2.0 : tmppt[1];
//  tmppt[2] = tmppt2[2] != tmppt[2] ? (tmppt2[2] + tmppt[2])/2.0 : tmppt[2];
//  delete tmpPtA;
//  delete tmpPtB;
//  delete tmppt2;
//  return tmppt;
//}

//void QuatToMat(float* q, float *&mat)
//{
//    mat[0] = 1 - 2*q[2]*q[2] - 2*q[3]*q[3];
//    mat[1] = 2*q[1]*q[2] + 2*q[0]*q[3];
//    mat[2] = 2*q[1]*q[3] - 2*q[0]*q[2];
//    mat[3] = 2*q[1]*q[2] - 2*q[0]*q[3];
//    mat[4] = 1 - 2*q[1]*q[1] - 2*q[3]*q[3];
//    mat[5] = 2*q[2]*q[3] + 2*q[0]*q[1];
//    mat[6] = 2*q[1]*q[3] + 2*q[0]*q[2];
//    mat[7] = 2*q[2]*q[3] - 2*q[0]*q[1];
//    mat[8] =  1 - 2*q[1]*q[1] - 2*q[2]*q[2];
//}

//void CalcCross(float *v1, float *v2, float *v3)
//{
//    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
//    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
//    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
//}

//void AngularCorrection(myBody* a, myBody* b, float* collPoint,float& collLen, int &collAxis)
//{
//  float* vec2 = a->GetCenter();
//  collPoint[collAxis] += collLen;
//  float vec1[3] = {collPoint[0] - a->vec2[0], collPoint[1] - a->vec2[1], collPoint[2] - a->vec2[2]};
//  float vh[3] = {0,0,0};
//  float* angVec = new float[3]();
//  vh[collAxis] = 1;
//  angVec[0] = vec1[1]*vh[2]-vec1[2]*vh[1];
//  angVec[1] = vec1[0]*vh[2]-vec1[2]*vh[0];
//  angVec[2] = vec1[0]*vh[1]-vec1[1]*vh[0];
//  AddAngularForce(-angVec[0], -angVec[1], -angVec[2],a);
//}

//bool CheckCollision(myBody* a, myBody* b, float &collisionLen, int &collisionAxis)
//{
//  GetOBB(&a);
//  GetOBB(&b);
//  float* matA = new float[9]();
//  QuatToMat(a->orientation, matA);
//  float* matB = new float[9]();
//  QuatToMat(b->orientation, matB);

//  float collPoint[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
//  float collPoint2[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
//  float collPoint3[3] = {FLT_MAX, FLT_MAX, FLT_MAX};

//  //SAT test for all axes
//  for (int in = 0; in < 3; in++)
//  {
//    float axis[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
//    if (!TestAxisSAT(obbA, obbB, axis, collPoint[in]))
//      return false;
//  }
//  for (int in = 0; in < 3; in++)
//  {
//    float axis[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
//    if (!TestAxisSAT(obbA, obbB, axis, collPoint2[in]))
//      return false;
//  }
//  for (int in = 0; in < 3; in++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
//      float axisB[3] = {matB[j*3], matB[j*3+1], matB[j*3+2]};
//      float axis[3];
//      CalcCross(axisA, axisB, axis);
//      if (!TestAxisSAT(obbA, obbB, axis, collPoint3[in]))
//        return false;
//    }
//  }
//  float *t2;
//  float collPoint4[4] = {FLT_MAX, FLT_MAX, FLT_MAX};
//  for (int in = 0; in < 3; in++)
//  {
//    float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
//    float axisB[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
//    PenetrationDepthCorrection(obbA, obbB, axisA, collisionLen, collisionAxis, in, collPoint4);
//    t2 = PenetrationDepthCorrection(obbA, obbB, axisB, collisionLen, collisionAxis, in, collPoint4);
//  }
//  collPoint4[0] = t2[0];
//  collPoint4[1] = t2[1];
//  collPoint4[2] = t2[2];
//  AngularCorrection(a, b, collPoint4, collisionLen, collisionAxis);

//  delete matA;
//  delete matB;
//  return true;
//}

//float4 QuaternionMul(float4 q1, float4 q2)
//{
//  return (float4)( q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
//                    q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
//                    q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
//                    q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
//}

__kernel void calcPoints(__global myBox* bodies,unsigned int numBodies, double timeStep, int first)
{
//    unsigned int i = get_global_id(0);
//    if(i>0) // without ground
//    {
//      for(int b = 0; b < numBodies; ++b)
//      {
//        if(i != b)
//        {
//          float collisionLen = FLT_MAX;
//          int collisionAxis = -1;
//          if(CheckCollision(&bodies[i], &bodies[b], collisionLen, collisionAxis))
//          {
//            bodies[i].center[collisionAxis] += collisionLen;
//            bodies[i].velocity[collisionAxis] *= -1;
//          }
//        }
//      }
//    }
}

__kernel void updatePoints(__global myBox* bodies,unsigned int numBodies, double timeStep, int first)
{
  unsigned int i = get_global_id(0);
//  if(bodies[i].isActive == true && first != 1)
//  {
//    // add gravity and airdrag
//    bodies[i].velocity[1] += bodies[i].weight*(-9.81)*timeStep; // added gravity


//    // Set new velocity
//    SetVelocity(bodies[i].velocity[0] * 0.99 / bodies[i].weight,
//                bodies[i].velocity[1] * 0.99,
//                bodies[i].velocity[2] * 0.99 / bodies[i].weight,
//                &bodies[i]);

//    // Update Position
//    bodies[i].center[0] += bodies[i].velocity[0]*timeStep;
//    bodies[i].center[1] += bodies[i].velocity[1]*timeStep;
//    bodies[i].center[2] += bodies[i].velocity[2]*timeStep;

//    if (bodies[i].angularForce[0] != 0.0f || bodies[i].angularForce[1] != 0.0f || bodies[i].angularForce[2] != 0.0f)
//    {
//        bodies[i].angularVelocity[0] -= bodies[i].angularForce[0]*timeStep;
//        bodies[i].angularVelocity[1] -= bodies[i].angularForce[1]*timeStep;
//        bodies[i].angularVelocity[2] -= bodies[i].angularForce[2]*timeStep;
//        bodies[i].angularVelocity[0] *= 0.99;
//        bodies[i].angularVelocity[1] *= 0.99;
//        bodies[i].angularVelocity[2] *= 0.99;
//    }
//  }
}

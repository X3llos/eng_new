#pragma OPENCL EXTENSION cl_khr_fp64 : enable
typedef struct
{
    bool isActive;
    uint type;
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

float AngleToRad(float deg)
{
  return M_PI/180*deg;
}

void SetCenter(float x, float y, float z, myBody *b)
{
  b->center[0] = x;
  b->center[1] = y;
  b->center[2] = z;
}

void SetVelocity(float vx, float vy, float vz, myBody *b)
{
  b->velocity[0] = vx;
  b->velocity[1] = vy;
  b->velocity[2] = vz;
}

void AddForce(float x, float y, float z, myBody *b)
{
  b->velocity[0] += x;
  b->velocity[1] += y;
  b->velocity[2] += z;
}

void AddAngularForce(float x, float y, float z, myBody *b)
{
  b->angularForce[0] += AngleToRad(x);
  b->angularForce[1] += AngleToRad(y);
  b->angularForce[2] += AngleToRad(z);
}

void SetOrientation(float ox, float oy, float oz, float ow, myBody *b)
{
  b->orientation[0] = ox;
  b->orientation[1] = oy;
  b->orientation[2] = oz;
  b->orientation[3] = ow;
}

void CalceOrient(myBody *b)
{
  float cos_z_2 = cos(0.5*b->angularVelocity[2]);
  float cos_y_2 = cos(0.5*b->angularVelocity[1]);
  float cos_x_2 = cos(0.5*b->angularVelocity[0]);

  float sin_z_2 = sin(0.5*b->angularVelocity[2]);
  float sin_y_2 = sin(0.5*b->angularVelocity[1]);
  float sin_x_2 = sin(0.5*b->angularVelocity[0]);

  b->orientation[0] = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
  b->orientation[1] = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
  b->orientation[2] = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
  b->orientation[3] = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;
}

void SetAngularVelocity(float vx, float vy, float vz, myBody *b)
{
  b->angularVelocity[0] = vx;
  b->angularVelocity[1] = vy;
  b->angularVelocity[2] = vz;
  CalceOrient(b);
}

void SetLengths(float x, float y, float z, myBody *b)
{
  b->lengths[0] = x;
  b->lengths[1] = y;
  b->lengths[2] = z;
}

void SetWeight(float w, myBody *b)
{
  b->weight = w;
}

float* QuaternionTransform(float* point, float* q)
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

void GetPoints(myBody *b)
{
  b->pts[0] = b->center[0]+b->lengths[0];
  b->pts[1] = b->center[1]+b->lengths[1];
  b->pts[2] = b->center[2]+b->lengths[2];
  b->pts[3] = b->center[0]-b->lengths[0];
  b->pts[4] = b->center[1]+b->lengths[1];
  b->pts[5] = b->center[2]+b->lengths[2];
  b->pts[6] = b->center[0]-b->lengths[0];
  b->pts[7] = b->center[1]-b->lengths[1];
  b->pts[8] = b->center[2]+b->lengths[2];
  b->pts[9] = b->center[0]+b->lengths[0];
  b->pts[10] = b->center[1]-b->lengths[1];
  b->pts[11] = b->center[2]+b->lengths[2];
  b->pts[12] = b->center[0]+b->lengths[0];
  b->pts[13] = b->center[1]-b->lengths[1];
  b->pts[14] = b->center[2]-b->lengths[2];
  b->pts[15] = b->center[0]-b->lengths[0];
  b->pts[16] = b->center[1]-b->lengths[1];
  b->pts[17] = b->center[2]-b->lengths[2];
  b->pts[18] = b->center[0]-b->lengths[0];
  b->pts[19] = b->center[1]+b->lengths[1];
  b->pts[20] = b->center[2]-b->lengths[2];
  b->pts[21] = b->center[0]+b->lengths[0];
  b->pts[22] = b->center[1]+b->lengths[1];
  b->pts[23] = b->center[2]-b->lengths[2];
}

void GetOBB(myBody *b)
{
  GetPoints(b);
  for(int i=0;i<8;i++)
  {
      float p[3] = {b->pts[i*3]-b->center[0],b->pts[i*3+1]-b->center[1], b->pts[i*3+2]-b->center[2]};
      QuaternionTransform(p, b->orientation);
      b->pts[i*3] = p[0]+b->center[0];
      b->pts[i*3+1] = p[1]+b->center[1];
      b->pts[i*3+2] = p[2]+b->center[2];
  }
}

float CalcDot(float *v1, float *v2)
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

bool TestAxisSAT(float* ptsA, float* ptsB, float* axis)//, float* collPoint)
{
  float tmpPtA[3];
  float tmpPtB[3];
  float minval1 = FLT_MAX;
  float maxval1 = FLT_MIN;
  float minval2 = FLT_MAX;
  float maxval2 = FLT_MIN;
  for( int i = 0 ; i < 8 ; i++ )
  {
    tmpPtA[0] = ptsA[i*3];
    tmpPtA[1] = ptsA[i*3+1];
    tmpPtA[2] = ptsA[i*3+2];
    tmpPtB[0] = ptsB[i*3];
    tmpPtB[1] = ptsB[i*3+1];
    tmpPtB[2] = ptsB[i*3+2];
    float dotVal = CalcDot(tmpPtA, axis);
    if( dotVal < minval1 )  minval1=dotVal;
    if( dotVal > maxval1 )  maxval1=dotVal;
    dotVal = CalcDot(tmpPtB, axis);
    if( dotVal < minval2 )  minval2=dotVal;
    if( dotVal > maxval2 )  maxval2=dotVal;
  }
  if(!((minval2 >= minval1 && minval2 <= maxval1) || (minval1 >= minval2 && minval1 <= maxval2)))
  {
    return false;
  }
  //*collPoint = maxval1 + minval1;
  return true;
}

void PenetrationDepthCorrection(float* ptsA, float* ptsB, float* axis,
                                        float* collLen, int* collAxis, int actAxis, float *collPoint, float* tmppt)
{
  float tmpPtA[3];
  float tmpPtB[3];
  float tmppt2[3];
  float minval1 = FLT_MAX;
  float maxval1 = -FLT_MAX;
  float minval2 = FLT_MAX;
  float maxval2 = -FLT_MAX;
  tmppt2[0] = FLT_MAX;

  float tmpcollLen;
  for( int i = 0 ; i < 8 ; i++ )
  {
    tmpPtA[0] = ptsA[i*3];
    tmpPtA[1] = ptsA[i*3+1];
    tmpPtA[2] = ptsA[i*3+2];
    tmpPtB[0] = ptsB[i*3];
    tmpPtB[1] = ptsB[i*3+1];
    tmpPtB[2] = ptsB[i*3+2];

    float dotVal = CalcDot(tmpPtA, axis);
    if( dotVal < minval1 )  minval1=dotVal;
    if( dotVal > maxval1 )  maxval1=dotVal;
    dotVal = CalcDot(tmpPtB, axis);
    if( dotVal < minval2 )  minval2=dotVal;
    if( dotVal > maxval2 )  maxval2=dotVal;

    if (fabs(tmpPtA[0]) == fabs(*collLen) || fabs(tmpPtA[1]) == fabs(*collLen) || fabs(tmpPtA[2]) == fabs(*collLen))
    {
      if (tmppt2[0] == FLT_MAX)
      {
        tmppt2[0] = tmpPtA[0];
        tmppt2[1] = tmpPtA[1];
        tmppt2[2] = tmpPtA[2];
      }
      else
      {
        tmppt[0] = tmpPtA[0];
        tmppt[1] = tmpPtA[1];
        tmppt[2] = tmpPtA[2];
      }
    }
  }
  tmpcollLen = maxval1 > maxval2 ? (maxval2 - minval1) : (maxval1 - minval2);
  if(fabs(*collLen) > fabs(tmpcollLen))
  {
    *collLen = tmpcollLen;
    *collAxis = actAxis;
  }
  tmppt[0] = tmppt2[0] != tmppt[0] ? (tmppt2[0] + tmppt[0])/2.0 : tmppt[0];
  tmppt[1] = tmppt2[1] != tmppt[1] ? (tmppt2[1] + tmppt[1])/2.0 : tmppt[1];
  tmppt[2] = tmppt2[2] != tmppt[2] ? (tmppt2[2] + tmppt[2])/2.0 : tmppt[2];
}

void QuatToMat(myBody* a, float* mat)
{
    mat[0] = 1 - 2*a->orientation[2]*a->orientation[2] - 2*a->orientation[3]*a->orientation[3];
    mat[1] = 2*a->orientation[1]*a->orientation[2] + 2*a->orientation[0]*a->orientation[3];
    mat[2] = 2*a->orientation[1]*a->orientation[3] - 2*a->orientation[0]*a->orientation[2];
    mat[3] = 2*a->orientation[1]*a->orientation[2] - 2*a->orientation[0]*a->orientation[3];
    mat[4] = 1 - 2*a->orientation[1]*a->orientation[1] - 2*a->orientation[3]*a->orientation[3];
    mat[5] = 2*a->orientation[2]*a->orientation[3] + 2*a->orientation[0]*a->orientation[1];
    mat[6] = 2*a->orientation[1]*a->orientation[3] + 2*a->orientation[0]*a->orientation[2];
    mat[7] = 2*a->orientation[2]*a->orientation[3] - 2*a->orientation[0]*a->orientation[1];
    mat[8] =  1 - 2*a->orientation[1]*a->orientation[1] - 2*a->orientation[2]*a->orientation[2];
}

void CalcCross(float *v1, float *v2, float *v3)
{
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void AngularCorrection(myBody* a, myBody* b, float* collPoint,float collLen, int collAxis)
{
  float* vec2 = a->center;
  collPoint[collAxis] += collLen;
  float3 vec1 = (float3)(collPoint[0] - vec2[0], collPoint[1] - vec2[1], collPoint[2] - vec2[2]);
  float vh[3];
  vh[0] = 0;
  vh[1] = 0;
  vh[2] = 0;
  float angVec[3];
  vh[collAxis] = 1;
  angVec[0] = vec1.y*vh[2]-vec1.z*vh[1];
  angVec[1] = vec1.x*vh[2]-vec1.z*vh[0];
  angVec[2] = vec1.x*vh[1]-vec1.y*vh[0];
  AddAngularForce(-angVec[0], -angVec[1], -angVec[2],a);
}

bool CheckCollision(myBody* a, myBody* b, float* collisionLen, int* collisionAxis)
{
  GetOBB(a);
  GetOBB(b);
  float matA[9];
  QuatToMat(a, &matA[0]);
  float matB[9];
  QuatToMat(b, &matB[0]);

  //SAT test for all axes
  for (int in = 0; in < 3; in++)
  {
    float axis[3];
    axis[0] = matA[in*3];
    axis[1] = matA[in*3+1];
    axis[2] = matA[in*3+2];
    if (!TestAxisSAT(&(a->pts[0]), &(b->pts[0]), &axis[0]))//, &collPoint[in]))
      return false;
  }
  for (int in = 0; in < 3; in++)
  {
    float axis[3];
    axis[0] = matB[in*3];
    axis[1] = matB[in*3+1];
    axis[2] = matB[in*3+2];
    if (!TestAxisSAT(&(a->pts[0]), &(b->pts[0]), &axis[0]))//, &collPoint2[in]))
      return false;
  }
  for (int in = 0; in < 3; in++)
  {
    for (int j = 0; j < 3; j++)
    {
      float axisA[3];
      axisA[0] = matA[in*3];
      axisA[1] = matA[in*3+1];
      axisA[2] = matA[in*3+2];
      float axisB[3];
      axisB[0] = matB[j*3];
      axisB[1] = matB[j*3+1];
      axisB[2] = matB[j*3+2];
      float axis[3];
      CalcCross(&axisA[0], &axisB[0], &axis[0]);
      if (!TestAxisSAT(&(a->pts[0]), &(b->pts[0]), &axis[0]))//, &collPoint3[in]))
        return false;
    }
  }
  float t2[3];
  float t1[3];
  float collPoint[3];
  collPoint[0] = FLT_MAX;
  collPoint[1] = FLT_MAX;
  collPoint[2] = FLT_MAX;
  for (int in = 0; in < 3; in++)
  {
      float axisA[3];
      axisA[0] = matA[in*3];
      axisA[1] = matA[in*3+1];
      axisA[2] = matA[in*3+2];
      float axisB[3];
      axisB[0] = matB[in*3];
      axisB[1] = matB[in*3+1];
      axisB[2] = matB[in*3+2];
    PenetrationDepthCorrection(&(a->pts[0]), &(b->pts[0]), axisA, collisionLen, collisionAxis, in, collPoint, t1);
    PenetrationDepthCorrection(&(a->pts[0]), &(b->pts[0]), axisB, collisionLen, collisionAxis, in, collPoint, t2);
  }
  collPoint[0] = t2[0];
  collPoint[1] = t2[1];
  collPoint[2] = t2[2];
  //AngularCorrection(a, b, collPoint, *collisionLen, *collisionAxis);

  return true;
}

float4 QuaternionMul(float4 q1, float4 q2)
{
  return (float4)( q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
                    q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
                    q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
                    q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
}

__kernel void calcPoints(__global myBody* bodies,unsigned int numBodies, double timeStep, int first)
{
    unsigned int i = get_global_id(0);
    if(i>0) // without ground
    {
      for(int b = 0; b < numBodies; ++b)
      {
        if(i != b)
        {
          float collisionLen = FLT_MAX;
          int collisionAxis = -1;
          myBody obA = bodies[i];
          myBody obB = bodies[b];
          if(CheckCollision(&obA, &obB, &collisionLen, &collisionAxis))
          {
            bodies[i].center[collisionAxis] += collisionLen;
            bodies[i].velocity[collisionAxis] *= -1;
          }

        }
      }
    }
}

__kernel void updatePoints(__global myBody* bodies,unsigned int numBodies, double timeStep, int first)
{
  unsigned int i = get_global_id(0);
  if(bodies[i].isActive == true && first != 1)
  {
    // add gravity and airdrag
    bodies[i].velocity[1] += bodies[i].weight*(-9.81)*timeStep; // added gravity


    // Set new velocity
    bodies[i].velocity[0] = bodies[i].velocity[0] * 0.98 / bodies[i].weight;
    bodies[i].velocity[1] = bodies[i].velocity[1] * 0.98;
    bodies[i].velocity[2] = bodies[i].velocity[2] * 0.98 / bodies[i].weight;

    // Update Position
    bodies[i].center[0] += bodies[i].velocity[0]*timeStep;
    bodies[i].center[1] += bodies[i].velocity[1]*timeStep;
    bodies[i].center[2] += bodies[i].velocity[2]*timeStep;

    if (bodies[i].angularForce[0] != 0.0f || bodies[i].angularForce[1] != 0.0f || bodies[i].angularForce[2] != 0.0f)
    {
        bodies[i].angularVelocity[0] -= bodies[i].angularForce[0]*timeStep;
        bodies[i].angularVelocity[1] -= bodies[i].angularForce[1]*timeStep;
        bodies[i].angularVelocity[2] -= bodies[i].angularForce[2]*timeStep;
        bodies[i].angularVelocity[0] *= 0.98;
        bodies[i].angularVelocity[1] *= 0.98;
        bodies[i].angularVelocity[2] *= 0.98;
    }
  }
}

#include "stdafx.h"

Solver::Solver()
{
}

Solver::~Solver()
{
}

int Solver::Init()
{
  device_id = NULL;
  context = NULL;
  command_queue = NULL;
  program = NULL;
  kernel = NULL;
  wasText = false;
  fileName = const_cast<char*>("./test_SAT.cl");
  fp = fopen(fileName, "r");
  if (!fp)
  {
    fprintf(stderr, "Failed to load kernel.\n");
    return 1;
  }
  source_str = (char*)malloc(MAX_SOURCE_SIZE);
  source_size = fread(source_str, 1, MAX_SOURCE_SIZE, fp);
  fclose(fp);
  return 0;
}

int Solver::Update(double delta, Body* points, int boxesSize, int first)
{
//  // Get platform and device information
//  ret = clGetPlatformIDs (1, &platform_id, &ret_num_platforms);
//  ret = clGetDeviceIDs( platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);

//  // Create OpenCL context
//  context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);

//  // Create Command Queue
//  command_queue = clCreateCommandQueue(context, device_id, 0, &ret);

//  // Create Memory Buffer
//    cl_a = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(myBox)*boxesSize, NULL, &ret);

//  // Copy the lists A and B to their respective memory buffers
//  ret = clEnqueueWriteBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBox)* boxesSize, points, 0, NULL, NULL);

//  // Create Kernel Program from the source
//  program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &ret);

//  // Build Kernel Program
//  ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);

//  // Create OpenCL Kernel
//  kernel = clCreateKernel(program, "calcPoints", &ret);

//  // Set OpenCL Kernel Parameters
//  ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&cl_a);
//  ret = clSetKernelArg(kernel, 1, sizeof(int), &boxesSize);
//  cl_double d = (cl_double)delta;
//  ret = clSetKernelArg(kernel, 2, sizeof(cl_double), &d);

//  ret = clSetKernelArg(kernel, 3, sizeof(cl_bool), &first);


//  size_t global_item_size = boxesSize; // Process the entire lists
//  ret = clEnqueueNDRangeKernel(command_queue, kernel, 1, NULL, &global_item_size, &global_item_size, 0, NULL, NULL);

//  // Copy results from the memory buffer
//  ret = clEnqueueReadBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBox)*boxesSize, points, 0, NULL, NULL);

//  // Finalization
//  ret = clFlush(command_queue);
//  ret = clFinish(command_queue);
//  ret = clReleaseKernel(kernel);
//  ret = clReleaseProgram(program);
//  ret = clReleaseMemObject(cl_a);
//  ret = clReleaseCommandQueue(command_queue);
//  ret = clReleaseContext(context);

  return 0;
}

void Solver::UpdateVelocity(Body* obj, double &timeStep)
{
  // add gravity and airdrag
  float* vel = obj->GetVelocity();
  vel[1] += obj->GetWeight()*(-9.81)*timeStep; // added gravity


  // Set new velocity
  obj->SetVelocity( vel[0] * AIRDRAG / obj->GetWeight(),
                    vel[1] * AIRDRAG,
                    vel[2] * AIRDRAG / obj->GetWeight());

  // Update Position
  float* cent = obj->GetCenter();
  vel = obj->GetVelocity();   //CHANGE THIS LATER
  obj->SetCenter(cent[0]+vel[0]*timeStep, cent[1]+vel[1]*timeStep, cent[2]+vel[2]*timeStep);
}

void Solver::UpdateAngularVelocity(Body* obj, double &timeStep)
{
  float* angVel = obj->GetAngularVelocity();
  float* angForce = obj->GetAngularForce();
  angVel[0] += angForce[0]*timeStep;
  angVel[1] += angForce[1]*timeStep;
  angVel[2] += angForce[2]*timeStep;
  obj->SetAngularVelocity(angVel[0],angVel[1],angVel[2]);
}

int Solver::UpdateCPU(double timeStep,std::vector<Body*> bodies, int first)
{
  for(int i=0; i<numBoxes; ++i)
  {
    if(bodies[i]->isActive == true && first != 1)
    {
      UpdateVelocity(bodies[i], timeStep);
      UpdateAngularVelocity(bodies[i], timeStep);
    }
  }
  for(int a=1; a<numBoxes; ++a) // without ground
  {
    for(int b = 0; b < numBoxes; ++b)
    {
      if(a != b)
      {
        float collisionLen = FLT_MAX;
        int collisionAxis = -1;
        if(CheckCollision(bodies[a], bodies[b], collisionLen, collisionAxis))
        {
          float* tmpvel = new float[3]();
          tmpvel = bodies[a]->GetCenter();

          tmpvel[collisionAxis] += collisionLen;
          tmpvel = bodies[a]->GetVelocity();
          tmpvel[collisionAxis] *= -1;
          bodies[a]->SetVelocity(tmpvel[0], tmpvel[1], tmpvel[2]);
          // deactivate nearly non-moving objects
//              if(std::abs(tmpvel[0]) < 0.01 && std::abs(tmpvel[1]) < 0.01 && std::abs(tmpvel[2]) < 0.01)
//                bodies[a]->isActive = false;
//              else
//                bodies[a]->isActive = true;
        }
      }
    }
  }
  return 0;
}

/*
//Old collision detection and velocity change code

//3 axis of A and 3 axis of B
//  for (int in = 0; in < 3; in++)
//  {
//    float axis[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
//    float ra = std::abs(CalcDot(lenA, axis));
//    float rb = std::abs(CalcDot(lenB, axis));
//    float distance2 = glm::length2(CalcDot(newCentB, axis));
//    if (distance2 >= glm::length2(ra + rb))
//    {
//      return false;
//    }
//  }
//  for (int in = 0; in < 3; in++)
//  {
//    float axis[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
//    float ra = std::abs(CalcDot(lenA, axis));
//    float rb = std::abs(CalcDot(lenB, axis));
//    float distance2 = glm::length2(CalcDot(newCentA, axis));
//    if (distance2 >= glm::length2(ra + rb))
//    {
//      return false;
//    }
//  }
//  for (int in = 0; in < 3; in++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
//      float axisB[3] = {matB[j*3], matB[j*3+1], matB[j*3+2]};
//      float axis[3];
//      CalcCross(axisA, axisB, axis);
//      float ra = std::abs(CalcDot(orientedLenA, axis));
//      float rb = std::abs(CalcDot(orientedLenB, axis));
//      float distance2 = glm::length2(CalcDot(newCentA, axis));
//      if (distance2 >= glm::length2(ra + rb) && distance2 != 0.0f)
//      {
//        return false;
//      }
//    }
//  }
// COLLISION REBOUND
//            if(isCollision == true)
//            {
//                float elasticity = 0.75;
//                bodies[i].Center.y -= bodies[i].Weight*(-9.81)*1*timeStep;
//                bodies[i].Velocity *= (-elasticity);
//                std::cout<<i<<" with "<<z<<std::endl;
//              //bodies[i].isActive = false;
//            }

    // handle rotations
    //if(GetLength(bodies[i].AngularVelocity) > 0.0)

//        glm::vec4 a = glm::vec4(sin(bodies[i].AngularVelocity.x/2.0f), 0, 0, cos(bodies[i].AngularVelocity.x/2.0f));
//        glm::vec4 b = glm::vec4(0, sin(bodies[i].AngularVelocity.y/2.0f), 0, cos(bodies[i].AngularVelocity.y/2.0f));
//        glm::vec4 c = glm::vec4(0, 0, sin(bodies[i].AngularVelocity.z/2.0f), cos(bodies[i].AngularVelocity.z/2.0f));
//        bodies[i].Orientation = QuaternionMul(QuaternionMul(a,b),c);
//        bodies[i].Orientation = glm::normalize(bodies[i].Orientation);
*/

bool Solver::TestAxisSAT(float* ptsA,float* ptsB, float* axis)// float &collLen, int &collAxis)
{
  float* tmpPtA = new float[3]();
  float* tmpPtB = new float[3]();
  float minval1 = FLT_MAX;
  float maxval1 = -FLT_MAX;
  float minval2 = FLT_MAX;
  float maxval2 = -FLT_MAX;
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
  delete tmpPtA;
  delete tmpPtB;
  if(!((minval2 >= minval1 && minval2 <= maxval1) || (minval1 >= minval2 && minval1 <= maxval2)))
  {
    return false;
  }

  return true;
}

bool Solver::CheckCollision(Body *a, Body *b, float &collisionLen, int &collisionAxis)
{
  float* obbA = a->GetOBB();
  float* obbB = b->GetOBB();
  float* matA = new float[9]();
  QuatToMat(a->GetOrientation(), matA);
  float* matB = new float[9]();
  QuatToMat(b->GetOrientation(), matB);

  //SAT test for all axes
  for (int in = 0; in < 3; in++)
  {
    float axis[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
    if (!TestAxisSAT(obbA, obbB, axis))//, collisionLen, collisionAxis))
      return false;
    //PenetrationDepthCorrection(obbA, obbB, axis, collisionLen, collisionAxis, in);
  }
  for (int in = 0; in < 3; in++)
  {
    float axis[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
    if (!TestAxisSAT(obbA, obbB, axis))
      return false;
  }
  for (int in = 0; in < 3; in++)
  {
    for (int j = 0; j < 3; j++)
    {
      float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
      float axisB[3] = {matB[j*3], matB[j*3+1], matB[j*3+2]};
      float axis[3];
      CalcCross(axisA, axisB, axis);
      if (!TestAxisSAT(obbA, obbB, axis))
        return false;
    }
  }

  for (int in = 0; in < 3; in++)
  {
    float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
    float axisB[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
    PenetrationDepthCorrection(obbA, obbB, axisA, collisionLen, collisionAxis, in);
    PenetrationDepthCorrection(obbA, obbB, axisB, collisionLen, collisionAxis, in);
  }

  delete matA;
  delete matB;
  return true;
}

void Solver::PenetrationDepthCorrection(float* ptsA,float* ptsB, float* axis, float &collLen, int &collAxis, int actAxis)
{
  float* tmpPtA = new float[3]();
  float* tmpPtB = new float[3]();
  float minval1 = FLT_MAX;
  float maxval1 = -FLT_MAX;
  float minval2 = FLT_MAX;
  float maxval2 = -FLT_MAX;

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
  }
  tmpcollLen = maxval1 > maxval2 ? (maxval2 - minval1) : (maxval1 - minval2);

  if(std::abs(collLen) > std::abs(tmpcollLen))
  {
    collLen = tmpcollLen;
    collAxis = actAxis;
  }
  delete tmpPtA;
  delete tmpPtB;

}

int Solver::Close()
{
  return 0;
}

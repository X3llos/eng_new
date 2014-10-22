#include "stdafx.h"

Solver::Solver()
{
  tmppt = new float[3]();
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
  fileName = const_cast<char*>("./calcPhys.cl");
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

void Solver::UpdateBodiesGPU(double timeStep, myBody* bodies, int first)
{
  // Get platform and device information
  ret = clGetPlatformIDs (1, &platform_id, &ret_num_platforms);
  ret = clGetDeviceIDs( platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);
  // Create OpenCL context
  context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);

  // Create Command Queue
  command_queue = clCreateCommandQueue(context, device_id, 0, &ret);

  // Create Memory Buffer
  cl_a = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(myBody)*numBoxes, NULL, &ret);

  // Copy the lists A and B to their respective memory buffers
  ret = clEnqueueWriteBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBody)*numBoxes, bodies, 0, NULL, NULL);

  // Create Kernel Program from the source
  program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &ret);

  // Build Kernel Program
  ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);

  // Create OpenCL Kernel
  kernel = clCreateKernel(program, "updatePoints", &ret);

  // Set OpenCL Kernel Parameters
  ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), &cl_a);
  int nb = numBoxes;
  ret = clSetKernelArg(kernel, 1, sizeof(cl_int), &nb);
  cl_double d = (cl_double)timeStep;
  ret = clSetKernelArg(kernel, 2, sizeof(cl_double), &d);

  ret = clSetKernelArg(kernel, 3, sizeof(cl_bool), &first);


  size_t global_item_size = numBoxes; // Process the entire lists
  ret = clEnqueueNDRangeKernel(command_queue, kernel, 1, NULL, &global_item_size, &global_item_size, 0, NULL, NULL);
  ret = clFinish(command_queue);
  // Copy results from the memory buffer
  ret = clEnqueueReadBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBody)*numBoxes, bodies, 0, NULL, NULL);
  // Finalization
  ret = clFlush(command_queue);

  ret = clReleaseKernel(kernel);
  ret = clReleaseProgram(program);
  ret = clReleaseMemObject(cl_a);
  ret = clReleaseCommandQueue(command_queue);
  ret = clReleaseContext(context);
}

int Solver::UpdateGPU(double timeStep, myBody* bodies, int first)
{
  UpdateBodiesGPU(timeStep, bodies, first);

  // Get platform and device information
  ret = clGetPlatformIDs (1, &platform_id, &ret_num_platforms);
  ret = clGetDeviceIDs( platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);
  // Create OpenCL context
  context = clCreateContext(NULL, 1, &device_id, NULL, NULL, &ret);

  // Create Command Queue
  command_queue = clCreateCommandQueue(context, device_id, 0, &ret);

  // Create Memory Buffer
  cl_a = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(myBody)*numBoxes, NULL, &ret);

  // Copy the lists A and B to their respective memory buffers
  ret = clEnqueueWriteBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBody)*numBoxes, bodies, 0, NULL, NULL);

  // Create Kernel Program from the source
  program = clCreateProgramWithSource(context, 1, (const char **)&source_str, (const size_t *)&source_size, &ret);

  // Build Kernel Program
  ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);

  // Create OpenCL Kernel
  kernel = clCreateKernel(program, "calcPoints", &ret);

  // Set OpenCL Kernel Parameters
  ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), &cl_a);
  int nb = numBoxes;
  ret = clSetKernelArg(kernel, 1, sizeof(cl_int), &nb);
  cl_double d = (cl_double)timeStep;
  ret = clSetKernelArg(kernel, 2, sizeof(cl_double), &d);

  ret = clSetKernelArg(kernel, 3, sizeof(cl_bool), &first);


  size_t global_item_size = numBoxes; // Process the entire lists
  ret = clEnqueueNDRangeKernel(command_queue, kernel, 1, NULL, &global_item_size, &global_item_size, 0, NULL, NULL);
  ret = clFinish(command_queue);
  // Copy results from the memory buffer
  ret = clEnqueueReadBuffer(command_queue, cl_a, CL_TRUE, 0,sizeof(myBody)*numBoxes, bodies, 0, NULL, NULL);
  // Finalization
  ret = clFlush(command_queue);

  ret = clReleaseKernel(kernel);
  ret = clReleaseProgram(program);
  ret = clReleaseMemObject(cl_a);
  ret = clReleaseCommandQueue(command_queue);
  ret = clReleaseContext(context);

  return 0;
}

void Solver::UpdateVelocity(Body* obj, double &timeStep)
{
  // add gravity and airdrag
  float* vel = obj->GetVelocity();
  vel[1] += obj->GetWeight()*(-9.81)*timeStep*0.1; // added gravity


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
  angVel[0] -= angForce[0]*timeStep;
  angVel[1] -= angForce[1]*timeStep;
  angVel[2] -= angForce[2]*timeStep;
  angVel[0] *= AIRDRAG;
  angVel[1] *= AIRDRAG;
  angVel[2] *= AIRDRAG;
  obj->SetAngularVelocity(angVel[0]*timeStep,angVel[1]*timeStep,angVel[2]*timeStep);
}

int Solver::UpdateCPU(double timeStep,std::vector<Body*> bodies, int first)
{
  for(int i=0; i<numBoxes; ++i)
  {
    if(bodies[i]->isActive == true && first != 1)
    {
      UpdateVelocity(bodies[i], timeStep);
      float* angForce = bodies[i]->GetAngularForce();
      if (angForce[0] != 0.0f || angForce[1] != 0.0f || angForce[2] != 0.0f)
      {
        UpdateAngularVelocity(bodies[i], timeStep);
      }
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

bool Solver::TestAxisSAT(float* ptsA, float* ptsB, float* axis, float &collPoint)
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
  collPoint = maxval1 + minval1;
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

  float collPoint[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float collPoint2[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float collPoint3[3] = {FLT_MAX, FLT_MAX, FLT_MAX};

  //SAT test for all axes
  for (int in = 0; in < 3; in++)
  {
    float axis[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
    if (!TestAxisSAT(obbA, obbB, axis, collPoint[in]))
      return false;
  }
  for (int in = 0; in < 3; in++)
  {
    float axis[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
    if (!TestAxisSAT(obbA, obbB, axis, collPoint2[in]))
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
      if (!TestAxisSAT(obbA, obbB, axis, collPoint3[in]))
        return false;
    }
  }
  float *t2;
  float collPoint4[4] = {FLT_MAX, FLT_MAX, FLT_MAX};
  for (int in = 0; in < 3; in++)
  {
    float axisA[3] = {matA[in*3], matA[in*3+1], matA[in*3+2]};
    float axisB[3] = {matB[in*3], matB[in*3+1], matB[in*3+2]};
    PenetrationDepthCorrection(obbA, obbB, axisA, collisionLen, collisionAxis, in, collPoint4);
    t2 = PenetrationDepthCorrection(obbA, obbB, axisB, collisionLen, collisionAxis, in, collPoint4);
  }
  collPoint4[0] = t2[0];
  collPoint4[1] = t2[1];
  collPoint4[2] = t2[2];
  AngularCorrection(a, b, collPoint4, collisionLen, collisionAxis);

  delete matA;
  delete matB;
  return true;
}

void Solver::AngularCorrection(Body *a, Body *b, float* collPoint,float& collLen, int &collAxis)
{
  float* vec2 = a->GetCenter();
  collPoint[collAxis] += collLen;
  float vec1[3] = {collPoint[0] - vec2[0], collPoint[1] - vec2[1], collPoint[2] - vec2[2]};
  float vh[3] = {0,0,0};
  float* angVec = new float[3]();
  vh[collAxis] = 1;
  angVec[0] = vec1[1]*vh[2]-vec1[2]*vh[1];
  angVec[1] = vec1[0]*vh[2]-vec1[2]*vh[0];
  angVec[2] = vec1[0]*vh[1]-vec1[1]*vh[0];

  //vec1[axis] *= std::abs((vec2[axis]-collPoint2[axis]))/(vec2[axis]-collPoint2[axis]);
  //vec1[axis] += (vec2[axis]-collPoint2[axis]);// * axis;
  //a->AddAngularForce(angVec[2],angVec[2],-(angVec[0])-(angVec[1]));

  //std::cout<<"vec "<<vec1[0]<<" "<<vec1[1]<<" "<<vec1[2]<<std::endl;
  //std::cout<<"angVec "<<angVec[0]<<" "<<angVec[1]<<" "<<angVec[2]<<std::endl;
  a->AddAngularForce(-angVec[0], -angVec[1], -angVec[2]);
}

float* Solver::PenetrationDepthCorrection(float* ptsA, float* ptsB, float* axis,
                                        float &collLen, int &collAxis, int actAxis, float *collPoint)
{
  float* tmpPtA = new float[3]();
  float* tmpPtB = new float[3]();
  float* tmppt2 = new float[3]();
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

    if (std::abs(tmpPtA[0]) == std::abs(collLen) || std::abs(tmpPtA[1]) == std::abs(collLen) || std::abs(tmpPtA[2]) == std::abs(collLen))
    {
      if (tmppt2[0] == FLT_MAX)
      {
        tmppt2[0] = tmpPtA[0];
        tmppt2[1] = tmpPtA[1];
        tmppt2[2] = tmpPtA[2];
        //std::cout<<"tmppt2 "<<tmppt2[0]<<" "<<tmppt2[1]<<" "<<tmppt2[2]<<std::endl;
      }
      else
      {
        tmppt[0] = tmpPtA[0];
        tmppt[1] = tmpPtA[1];
        tmppt[2] = tmpPtA[2];
        //std::cout<<"tmppt "<<tmppt[0]<<" "<<tmppt[1]<<" "<<tmppt[2]<<std::endl;
      }
    }
  }
  tmpcollLen = maxval1 > maxval2 ? (maxval2 - minval1) : (maxval1 - minval2);
  //std::cout<<"val1 "<<minval1<<" "<<maxval1<<" val2 "<<minval2<<" "<<maxval2<<
  //" ||min1 "<<maxval2 - minval1<<" min2 "<<maxval1 - minval2<<std::endl;
  if(std::abs(collLen) > std::abs(tmpcollLen))
  {
    collLen = tmpcollLen;
    collAxis = actAxis;
  }
  tmppt[0] = tmppt2[0] != tmppt[0] ? (tmppt2[0] + tmppt[0])/2.0 : tmppt[0];
  tmppt[1] = tmppt2[1] != tmppt[1] ? (tmppt2[1] + tmppt[1])/2.0 : tmppt[1];
  tmppt[2] = tmppt2[2] != tmppt[2] ? (tmppt2[2] + tmppt[2])/2.0 : tmppt[2];
  //std::cout<<"tmppt out "<<tmppt[0]<<" "<<tmppt[1]<<" "<<tmppt[2]<<std::endl;
  delete tmpPtA;
  delete tmpPtB;
  delete tmppt2;
  return tmppt;
}

int Solver::Close()
{
  return 0;
}

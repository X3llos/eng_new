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

cl_float4 Solver::QuaternionMul(cl_float4 q1, cl_float4 q2)
{
  cl_float4 tmp = {q1.s[3]*q2.s[0] + q1.s[0]*q2.s[3] + q1.s[1]*q2.s[2] - q1.s[2]*q2.s[1],
               q1.s[3]*q2.s[1] + q1.s[1]*q2.s[3] + q1.s[2]*q2.s[0] - q1.s[0]*q2.s[2],
               q1.s[3]*q2.s[2] + q1.s[2]*q2.s[3] + q1.s[0]*q2.s[1] - q1.s[1]*q2.s[0],
               q1.s[3]*q2.s[3] - q1.s[0]*q2.s[0] - q1.s[1]*q2.s[1] - q1.s[2]*q2.s[2]};
  return tmp;
}

glm::vec4 Solver::QuaternionMul(glm::vec4 q1, glm::vec4 q2)
{
  glm::vec4 tmp = glm::vec4(q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
               q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
               q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
               q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
  return tmp;
}


cl_float3 Solver::QuaternionTransform(cl_float3 p,  cl_float4 q)
{
	float xx = q.s[0] * q.s[0], yy = q.s[1] * q.s[1], zz = q.s[2] * q.s[2],
		xy = q.s[0] * q.s[1], xz = q.s[0] * q.s[2],
		yz = q.s[1] * q.s[2], wx = q.s[3] * q.s[0],
		wy = q.s[3] * q.s[1], wz = q.s[3] * q.s[2];
	cl_float3 tmp = {(1.0f - 2.0f * ( yy + zz )) * p.s[0] + (2.0f * ( xy - wz )) * p.s[1] + (2.0f * ( xz + wy )) * p.s[2],
			 (2.0f * ( xy + wz )) * p.s[0] + (1.0f - 2.0f * ( xx + zz )) * p.s[1] + (2.0f * ( yz - wx )) * p.s[2],
			 (2.0f * ( xz - wy )) * p.s[0] + (2.0f * ( yz + wx )) * p.s[1] + (1.0f - 2.0f * ( xx + yy )) * p.s[2]};
	return tmp;
}

glm::vec3 Solver::QuaternionTransform(glm::vec3 p,  glm::vec4 q)
{
	float xx = q.x * q.w, yy = q.y * q.y, zz = q.z * q.z,
		xy = q.x * q.y, xz = q.x * q.z,
		yz = q.y * q.z, wx = q.w * q.x,
		wy = q.w * q.y, wz = q.w * q.z;
	glm::vec3 tmp = glm::vec3((1.0f - 2.0f * ( yy + zz )) * p.x + (2.0f * ( xy - wz )) * p.y + (2.0f * ( xz + wy )) * p.z,
			 (2.0f * ( xy + wz )) * p.x + (1.0f - 2.0f * ( xx + zz )) * p.y + (2.0f * ( yz - wx )) * p.z,
			 (2.0f * ( xz - wy )) * p.x + (2.0f * ( yz + wx )) * p.y + (1.0f - 2.0f * ( xx + yy )) * p.z);
	return tmp;
}

float Solver::getDistance(cl_float3 v1, cl_float3 v2)
{
	float dx = v2.s[0] - v1.s[0];
	float dy = v2.s[1] - v1.s[1];
	float dz = v2.s[2] - v1.s[2];

	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float Solver::getDistance(glm::vec3 v1, glm::vec3 v2)
{
	float dx = v2.x - v1.x;
	float dy = v2.y - v1.y;
	float dz = v2.z - v1.z;

	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

cl_float3 Solver::Radians(cl_float3 d)
{
  cl_float3 tmp = {d.s[0] * ONE_DEG_IN_RAD, d.s[1] * ONE_DEG_IN_RAD, d.s[2] * ONE_DEG_IN_RAD};
  return tmp;
} 

glm::vec3 Solver::Radians(glm::vec3 d)
{
  glm::vec3 tmp = glm::vec3(d.x * ONE_DEG_IN_RAD, d.y * ONE_DEG_IN_RAD, d.z * ONE_DEG_IN_RAD);
  return tmp;
}

float Solver::GetLength(cl_float3 vec)
{
  return std::sqrt((vec.s[0] * vec.s[0]) + (vec.s[1] * vec.s[1]) + (vec.s[2] * vec.s[2]));
}

float Solver::GetLength(glm::vec3 vec)
{
  return std::sqrt((vec.x * vec.x) + (vec.y * vec.y) + (vec.z * vec.z));
}

float Solver::GetLength4(cl_float4 vec)
{
  return std::sqrt((vec.s[0] * vec.s[0]) + (vec.s[1] * vec.s[1]) + (vec.s[2] * vec.s[2]) + (vec.s[3] * vec.s[3]));
}

cl_float4 Solver::Normalize(cl_float4 vec)
{
  float len = GetLength(vec);
  if(len>0.0)
    len = 1/len;
  else
    len = 1;
  cl_float4 tmp = {vec.s[0]*len, vec.s[1]*len, vec.s[2]*len, vec.s[3]*len};
  return tmp;
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
  //TODO
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
          //Get OBB from object 'z' and 'i'
          //Check for collision
          //Add velocity and angular forces to object 'i'
          if(CheckCollision(bodies[a], bodies[b]))
             bodies[a]->isActive = false;

      }
    }
  }

  return 0;
}

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

bool Solver::TestAxisSAT(float* ptsA,float* ptsB, float* axis)
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

bool Solver::CheckCollision(Body *a, Body *b)
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
    if (!TestAxisSAT(obbA, obbB, axis))
      return false;
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

  delete matA;
  delete matB;
  return true;
}

int Solver::Close()
{
  return 0;
}

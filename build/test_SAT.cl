#pragma OPENCL EXTENSION cl_khr_fp64 : enable
typedef struct
{
  float3 Center;
  float3 Lengths;
  float3 Points[8];
  float4 Orientation;
  float3 Velocity;
  float3 AngularVelocity;
  float Weight;
  bool isActive;
} myBox;

float4 QuaternionMul(float4 q1, float4 q2)
{
  return (float4)( q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y,
                    q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z,
                    q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x,
                    q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z);
}

float3 QuaternionTransform(float3 p,  float4 q)
{
  float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z,
        xy = q.x * q.y, xz = q.x * q.z,
        yz = q.y * q.z, wx = q.w * q.x,
        wy = q.w * q.y, wz = q.w * q.z;

    return (float3)((1.0f - 2.0f * ( yy + zz )) * p.x + (2.0f * ( xy - wz )) * p.y + (2.0f * ( xz + wy )) * p.z,
                    (2.0f * ( xy + wz )) * p.x + (1.0f - 2.0f * ( xx + zz )) * p.y + (2.0f * ( yz - wx )) * p.z,
                    (2.0f * ( xz - wy )) * p.x + (2.0f * ( yz + wx )) * p.y + (1.0f - 2.0f * ( xx + yy )) * p.z);
}

__kernel void calcPoints(__global myBox* bodies,unsigned int numBodies, double timeStep, int first)
{
    unsigned int i = get_global_id(0);
    if(bodies[i].isActive == true)
    {
    if(first != 1)
    {
        float3 rotatedBox[8];
        for(unsigned int z = 0; z < numBodies; ++z)
        {
            //check with floor ( plane)
            if(z == 0)
            {
                float dif = distance(bodies[z].Center.y, bodies[i].Center.y);
                float dist = bodies[z].Lengths.y + bodies[i].Lengths.y;
                if (dif <= dist)
                {
                    bodies[i].Center.y += dist - dif;
                    bodies[i].Velocity.y *= -0.5;
                    if (bodies[i].Velocity.y < 0.1)
                        bodies[i].isActive = false;
                }
            }

            //check with boxes
            else if(z != i)
            {
                /// Calculate min/max of boxes
                float3 min1 = QuaternionTransform((-1)*bodies[i].Lengths, bodies[z].Orientation);
                float3 max1 = QuaternionTransform(bodies[i].Lengths, bodies[z].Orientation);
                float3 min2 = bodies[z].Center - bodies[i].Center;
                float3 max2 = bodies[z].Center - bodies[i].Center;
                for(int a=0; a<8; ++a)
                {
                    float3 differece = bodies[z].Points[a] - bodies[i].Center;
                    float3 tmpPointi = differece;//QuaternionTransform(differece, bodies[i].Orientation);
                    min2.x = min(min2.x, tmpPointi.x);
                    min2.y = min(min2.y, tmpPointi.y);
                    min2.z = min(min2.z, tmpPointi.z);
                    max2.x = max(max2.x, tmpPointi.x);
                    max2.y = max(max2.y, tmpPointi.y);
                    max2.z = max(max2.z, tmpPointi.z);
                }
                if (  (min2.x < max1.x && min1.x < max2.x) &&
                      (min2.y < max1.y && min1.y < max2.y) &&
                      (min2.z < max1.z && min1.z < max2.z)
                      )
                {
                    float minPush = MAXFLOAT;
                    float m1 =   min1.x < min2.x ? max1.x-min2.x : max2.x-min1.x;
                    float m2 =   min1.y < min2.y ? max1.y-min2.y : max2.y-min1.y;
                    float m3 =   min1.z < min2.z ? max1.z-min2.z : max2.z-min1.z;
                    minPush = min(minPush, m1);
                    minPush = min(minPush, m2);
                    minPush = min(minPush, m3);
                    float elasticity = 0.75;
                    if(minPush == m3)
                    {
                        bodies[i].Center.z += m3;
                        bodies[i].Velocity.z *= (-1)*elasticity;
                    }
                    else if(minPush == m2)
                    {
                        bodies[i].Center.y += m2;
                        bodies[i].Velocity.y = 2*bodies[i].Weight*bodies[i].Velocity.y/(bodies[i].Weight + bodies[z].Weight);
                        bodies[i].Velocity.y *= (-1)*elasticity;
                    }
                    else
                    {
                        bodies[i].Center.x += m1;
                        bodies[i].Velocity.x *= (-1)*elasticity;
                    }
                    //bodies[i].Center -=bodies[i].Velocity*(float3)(timeStep);
                    if (fabs(bodies[i].Velocity.y) < 1 && fabs(bodies[i].Velocity.x) < 1 && fabs(bodies[i].Velocity.z) < 1)
                        bodies[i].isActive = true; //should be false
                }
                if( (/*sqrt(2.0)**/(bodies[z].Lengths.x+bodies[i].Lengths.x))> fast_distance(bodies[z].Center, bodies[i].Center) && z > 0)
                    bodies[z].isActive = true;
            }
        }

    }
    else
    {
         bodies[i].AngularVelocity = radians(bodies[i].AngularVelocity);
         float4 a = (float4)(sin(bodies[i].AngularVelocity.x/2.0f), 0, 0, cos(bodies[i].AngularVelocity.x/2.0f));
         float4 b = (float4)(0, sin(bodies[i].AngularVelocity.y/2.0f), 0, cos(bodies[i].AngularVelocity.y/2.0f));
         float4 c = (float4)(0, 0, sin(bodies[i].AngularVelocity.z/2.0f), cos(bodies[i].AngularVelocity.z/2.0f));
         bodies[i].Orientation = QuaternionMul(QuaternionMul(a,b),c);
         bodies[i].Orientation = normalize(bodies[i].Orientation);
         bodies[i].AngularVelocity = (float3)(0.0);
    }
    if (i!=0)
    {
        float airdrag = 0.8;
        bodies[i].Velocity.y += bodies[i].Weight*(-9.81)*airdrag*timeStep; // add gravity and airdrag
        bodies[i].Center += bodies[i].Velocity*(float3)(timeStep); // calculate new position of center
    }
    // handle rotations
    if(length(bodies[i].AngularVelocity))
    {
        float4 a = (float4)(sin(bodies[i].AngularVelocity.x/2.0f), 0, 0, cos(bodies[i].AngularVelocity.x/2.0f));
        float4 b = (float4)(0, sin(bodies[i].AngularVelocity.y/2.0f), 0, cos(bodies[i].AngularVelocity.y/2.0f));
        float4 c = (float4)(0, 0, sin(bodies[i].AngularVelocity.z/2.0f), cos(bodies[i].AngularVelocity.z/2.0f));
        bodies[i].Orientation = QuaternionMul(QuaternionMul(a,b),c);
        bodies[i].Orientation = normalize(bodies[i].Orientation);
    }
    //final points calculation
    bodies[i].Points[0] = QuaternionTransform((float3)(bodies[i].Lengths.x, bodies[i].Lengths.y, bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[1] = QuaternionTransform((float3)(-bodies[i].Lengths.x, bodies[i].Lengths.y, bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[2] = QuaternionTransform((float3)(-bodies[i].Lengths.x, -bodies[i].Lengths.y, bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[3] = QuaternionTransform((float3)(bodies[i].Lengths.x, -bodies[i].Lengths.y, bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[4] = QuaternionTransform((float3)(bodies[i].Lengths.x, -bodies[i].Lengths.y, -bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[5] = QuaternionTransform((float3)(-bodies[i].Lengths.x, -bodies[i].Lengths.y, -bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[6] = QuaternionTransform((float3)(-bodies[i].Lengths.x, bodies[i].Lengths.y, -bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    bodies[i].Points[7] = QuaternionTransform((float3)(bodies[i].Lengths.x, bodies[i].Lengths.y, -bodies[i].Lengths.z), bodies[i].Orientation) + bodies[i].Center;
    }
}

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

float4 getMin(float3* points)
{
float4 min;
min.x = points[0].x;
min.y = points[0].y;
min.z = points[0].z;
min.w = 0;
return min;
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
                bodies[i].Center.y += dist - dif;//bodies[i].Velocity*(float3)(timeStep);
                bodies[i].Velocity.y *= -0.5;
                if (bodies[i].Velocity.y < 0.1)
                    bodies[i].isActive = false;
            }
        }

        //check with boxes
        else if(z != i)
            {
            for(int a=0; a<8; ++a)
            {
                if ((bodies[z].Points[a].x > (bodies[i].Center.x - bodies[i].Lengths.x)) && (bodies[z].Points[a].x <= (bodies[i].Center.x + bodies[i].Lengths.x)) &&
                    (bodies[z].Points[a].y > (bodies[i].Center.y - bodies[i].Lengths.y)) && (bodies[z].Points[a].y <= (bodies[i].Center.y + bodies[i].Lengths.y)) &&
                    (bodies[z].Points[a].z > (bodies[i].Center.z - bodies[i].Lengths.z)) && (bodies[z].Points[a].z <= (bodies[i].Center.z + bodies[i].Lengths.z)))
                {
                float dif1 = distance(bodies[z].Center.x, bodies[i].Center.x);
                float dif2 = distance(bodies[z].Center.y, bodies[i].Center.y);
                float dif3 = distance(bodies[z].Center.z, bodies[i].Center.z);

                float3 min = (float3)(0,0,0);
                    if (dif1 < dif2)
                      if (dif1 < dif3)
                            min.x = -1;
                        else
                            min.z = -1;
                    else
                         if (dif2<dif3)
                            min.y = -1;
                        else
                            min.z = -1;
                    bodies[i].Center -=min*bodies[i].Velocity*(float3)(timeStep);

                    //bodies[i].Center -= bodies[i].Velocity*(float3)(timeStep);

                    float dif = distance(bodies[z].Center.y, bodies[i].Center.y);
                    float dist = bodies[z].Lengths.y + bodies[i].Lengths.y;
                    bodies[i].Velocity.y *= -0.5;
                    if(bodies[z].Center.y <= bodies[i].Center.y)
                    {
                        bodies[i].Center.y += dist - dif;
                    }
                    else
                    {
                        bodies[i].Center.y -= dist - dif;
                    }

                    dif = distance(bodies[z].Center.x, bodies[i].Center.x);
                    dist = bodies[z].Lengths.x + bodies[i].Lengths.x;
                    if(bodies[z].Center.x < bodies[i].Center.x)
                    {
                        bodies[i].Center.x += dist - dif;
                    }
                    else if(bodies[z].Center.x > bodies[i].Center.x)
                    {
                        bodies[i].Center.x -= dist - dif;
                    }
                    bodies[i].Velocity.x *= -0.5;

                    dif = distance(bodies[z].Center.z, bodies[i].Center.z);
                    dist = bodies[z].Lengths.z + bodies[i].Lengths.z;
                    if(bodies[z].Center.z < bodies[i].Center.z)
                    {
                        bodies[i].Center.z += dist - dif;
                    }
                    else if(bodies[z].Center.z > bodies[i].Center.z)
                    {
                        bodies[i].Center.z -= dist - dif;
                    }
                    bodies[i].Velocity.z *= -0.5;
                break;
                if (fabs(bodies[i].Velocity.y) < 0.1 && fabs(bodies[i].Velocity.x) < 0.1 && fabs(bodies[i].Velocity.z) < 0.1)
                    bodies[i].isActive = false;
                }
            }

            if( (/*sqrt(2.0)**/(bodies[z].Lengths.x+bodies[i].Lengths.x))> fast_distance(bodies[z].Center, bodies[i].Center) && z > 0)
                bodies[z].isActive = true;

            }
        }
        if (i!=0)
        {
            bodies[i].Velocity.y += bodies[i].Weight*(-9.81)*timeStep; // add gravity
            bodies[i].Center += bodies[i].Velocity*(float3)(timeStep); // calculate new position of center
        }
    }

    // handle rotations
    float4 a = (float4)(sin(bodies[i].AngularVelocity.x/2.0f), 0, 0, cos(bodies[i].AngularVelocity.x/2.0f));
    float4 b = (float4)(0, sin(bodies[i].AngularVelocity.y/2.0f), 0, cos(bodies[i].AngularVelocity.y/2.0f));
    float4 c = (float4)(0, 0, sin(bodies[i].AngularVelocity.z/2.0f), cos(bodies[i].AngularVelocity.z/2.0f));
    bodies[i].Orientation = QuaternionMul(QuaternionMul(a,b),c);
    bodies[i].Orientation = normalize(bodies[i].Orientation);

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

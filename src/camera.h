#ifndef CAMERA_H
#define CAMERA_H
#include "stdafx.h"
class Camera
{
public:
  ~Camera();
  static Camera& GetInstance()
  {
    static Camera instance;
    return instance;
  }
  void setPos(float newPos);
  float getPos();
  glm::mat4 getViewMat();
  float* getProjMat();
  void Update(float delta, GLFWwindow*);
  void move(float vec);

private:
  Camera();
   Camera(const Camera&);
  glm::vec3 pos;
  bool change;
  float cam_speed;
  float cam_yaw_speed;
  float cam_yaw;
  glm::mat4 T;
  glm::mat4 R;
  glm::mat4 view_mat;
  float proj_mat[16];
  glm::quat quaternion;
};

#endif // CAMERA_H

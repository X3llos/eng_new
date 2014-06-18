#include "stdafx.h"

Camera::Camera()
{
  change = false;
  cam_speed = 2.0f; // 2 unit per second
  cam_yaw_speed = 20.0f; // 20 degrees per second
  pos = glm::vec3(0.0f,4.0f,20.0f);
  cam_yaw = 0.0f; // y-rotation in degrees
  T = glm::translate(glm::mat4(), glm::vec3(-pos[0], -pos[1], -pos[2]));
  R = glm::rotate(glm::mat4(), -cam_yaw, glm::vec3(0, 1, 0)); //rotate Y axis only
  view_mat = R * T;

// input variables
float n = 0.1f; // clipping plane
float f = 100.0f; // clipping plane
float fov = 67.0f * ONE_DEG_IN_RAD; // convert 67 degrees to radians
float aspect = (float)WIDTH / (float)HEIGHT; // aspect ratio
// matrix components
float range = tan (fov * 0.5f) * n;
float Sx = (2.0f * n) / (range * aspect + range * aspect);
float Sy = n / range;
float Sz = -(f + n) / (f - n);
float Pz = -(2.0f * f * n) / (f - n);
proj_mat[0] = Sx;
proj_mat[1] = 0.0f;
proj_mat[2] = 0.0f;
proj_mat[3] = 0.0f;
proj_mat[4] = 0.0f;
proj_mat[5] = Sy;
proj_mat[6] = 0.0f;
proj_mat[7] = 0.0f;
proj_mat[8] = 0.0f;
proj_mat[9] = 0.0f;
proj_mat[10] = Sz;
proj_mat[11] = -1.0f;
proj_mat[12] = 0.0f;
proj_mat[13] = 0.0f;
proj_mat[14] = Pz;
proj_mat[15] = 0.0f;
}

Camera::~Camera()
{

}

void Camera::setPos(float newPos)
{

}

float Camera::getPos()
{
  return 0.0f;
}

void Camera::move(float vec)
{

}

glm::mat4 Camera::getViewMat()
{
  return view_mat;
}

float* Camera::getProjMat()
{
  return proj_mat;
}

void Camera::Update(float elapsed_seconds, GLFWwindow* window)
{
  Renderer::GetInstance().getWindow();
  // control keys
  bool cam_moved = false;

  //speed up
  if (glfwGetKey (window, GLFW_KEY_SPACE)) {
    cam_speed *=5;
    cam_yaw_speed *=3;
  }
  if (glfwGetKey (window, GLFW_KEY_A)) {
    pos[0] -= cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_D)) {
    pos[0] += cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_PAGE_UP)) {
    pos[1] += cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_PAGE_DOWN)) {
    pos[1] -= cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_W)) {
    pos[2] -= cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_S)) {
    pos[2] += cam_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_LEFT)) {
   cam_yaw += cam_yaw_speed * elapsed_seconds;
    cam_moved = true;
  }
  if (glfwGetKey (window, GLFW_KEY_RIGHT)) {
    cam_yaw -= cam_yaw_speed * elapsed_seconds;
    cam_moved = true;
  }
  // update view matrix
  if (cam_moved) {
    T = glm::translate(glm::mat4(), glm::vec3(-pos[0], -pos[1], -pos[2])); // cam translation
    R = glm::rotate(glm::mat4(), -cam_yaw, glm::vec3(0,1,0)); //
    view_mat = R * T;
  }
  cam_speed = 2.0;
  cam_yaw_speed = 20.0;
}

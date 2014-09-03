#ifndef RENDERER_H
#define RENDERER_H
#include "stdafx.h"
class Renderer
{
public:
  ~Renderer();

  int Init();
  int Init(std::vector <Box>*);
  int Init(Body*);
  int Update(double,std::vector<Body*>);
  int UpdateGPU(double elapsed_seconds,myBody* bodies);
  int Close();
  static Renderer& GetInstance()
  {
    static Renderer instance;
    return instance;
  }
  GLFWwindow* getWindow();
  void setWireframe(bool);
  glm::vec3* cl_float3ToVec3(cl_float3* points);

private:

  Renderer();
  Renderer(const Renderer&);
  GLFWwindow* window;
  //Body* boxes2;
  glm::vec3 tmppts[8];

  unsigned int vbo;
  unsigned int vao;
  unsigned int shader_programme;
  unsigned int vs;
  unsigned int fs;

  double previous_seconds;

  static GLuint indices[24];
  static glm::vec3 colors[8];

  //data;
};

#endif // RENDERER_H

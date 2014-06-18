#include "stdafx.h"

//GLuint Renderer::indices[24] = {
//    2, 0, 1, 3,                 // Front face
//    7, 4, 5, 6,                 // Back face
//    6, 5, 2, 0,                 // Left face
//    7, 1, 3, 4,                 // Right face
//    7, 6, 0, 1,                 // Top face
//    3, 2, 5, 4,                 // Bottom face
//};

GLuint Renderer::indices[24] = {
      0, 1, 2, 3,                 // Front face
      7, 4, 5, 6,                 // Back face
      6, 5, 2, 1,                 // Left face
      7, 0, 3, 4,                 // Right face
      7, 6, 1, 0,                 // Top face
      3, 2, 5, 4,                 // Bottom face
  };

glm::vec3 Renderer::colors[8] = {
    glm::vec3( 1, 1, 1 ), // 0
    glm::vec3( 0, 1, 1 ), // 1
    glm::vec3( 0, 0, 1 ), // 2
    glm::vec3( 1, 0, 1 ), // 3
    glm::vec3( 1, 0, 0 ), // 4
    glm::vec3( 0, 0, 0 ), // 5
    glm::vec3( 0, 1, 0 ), // 6
    glm::vec3( 1, 1, 0 ) // 7
};

Renderer::Renderer()
{

}

Renderer::~Renderer()
{

}

int Renderer::Init()
{
  // start GL context and O/S window using the GLFW helper library
    if (!glfwInit ())
    {
      fprintf (stderr, "ERROR: could not start GLFW3\n");
      return 1;
    }
    window = glfwCreateWindow (WIDTH, HEIGHT, "GPU Rigid body sim", NULL, NULL);
    if (!window) {
      fprintf (stderr, "ERROR: could not open window with GLFW3\n");
      glfwTerminate();
      return 1;
    }
    glfwMakeContextCurrent (window);

    // start GLEW extension handler
    glewInit ();

    // get version info
    const GLubyte* renderer = glGetString (GL_RENDERER); // get renderer string
    const GLubyte* version = glGetString (GL_VERSION); // version as a string
    printf ("Renderer: %s\n", renderer);
    printf ("OpenGL version supported %s\n", version);

    // tell GL to only draw onto a pixel if the shape is closer to the viewer
    glEnable (GL_DEPTH_TEST); // enable depth-testing
    glDepthFunc (GL_LESS); // depth-testing interprets a smaller value as "closer"

    const char* vertex_shader =
    "#version 400\n"
    "uniform mat4 view, proj;"
    //"in vec3 vp;"
    "layout(location = 0) in vec3 vp;"
    "layout(location = 1) in vec3 vertex_colour;"
    "out vec3 colour;"
    "void main () {"
    "colour = vertex_colour;"
    "gl_Position = proj * view * vec4 (vp, 1.0);"
    "}";

    const char* fragment_shader =
    "#version 400\n"
    "in vec3 colour;"
    "out vec4 frag_colour;"
    "void main () {"
    //"  frag_colour = vec4 (0.0, 0.0, 1.0, 1.0);"
    "  frag_colour = vec4 (colour, 1.0);"
    "}";

    vs = glCreateShader (GL_VERTEX_SHADER);
    glShaderSource (vs, 1, &vertex_shader, NULL);
    glCompileShader (vs);
    fs = glCreateShader (GL_FRAGMENT_SHADER);
    glShaderSource (fs, 1, &fragment_shader, NULL);
    glCompileShader (fs);

    shader_programme = glCreateProgram ();
    glAttachShader (shader_programme, fs);
    glAttachShader (shader_programme, vs);

    // insert location binding code here
    glBindAttribLocation (shader_programme, 0, "vertex_position");
    glBindAttribLocation (shader_programme, 1, "vertex_colour");

    glLinkProgram (shader_programme);

    previous_seconds = glfwGetTime();
  return 0;
}


int Renderer::Init(Body* box)
{
  return -1;
}

int Renderer::Init(std::vector<Box>* box)
{
  return Init();
}

glm::vec3* Renderer::cl_float3ToVec3(cl_float3* points)
{
  for(int i=0;i<8;++i)
    tmppts[i] = glm::vec3(points[i].s[0], points[i].s[1], points[i].s[2] );
  return tmppts;
}

int Renderer::Update(double elapsed_seconds,std::vector<Body*> bodies)
{
  Camera::GetInstance().Update(elapsed_seconds, window);

  // wipe the drawing surface clear
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  int view_mat_location = glGetUniformLocation (shader_programme, "view");
  glUseProgram (shader_programme);
  glm::mat4 tmpmat = Camera::GetInstance().getViewMat();
  glUniformMatrix4fv (view_mat_location, 1, GL_FALSE, glm::value_ptr(tmpmat));
  int proj_mat_location = glGetUniformLocation (shader_programme, "proj");
  glUseProgram (shader_programme);
  glUniformMatrix4fv (proj_mat_location, 1, GL_FALSE, Camera::GetInstance().getProjMat());
  glBindVertexArray (vao);

  for(int i = 0;i<numBoxes;++i)
  {
      glEnableClientState( GL_COLOR_ARRAY );
      glEnableClientState( GL_VERTEX_ARRAY );

      vbo = 0;
      glGenBuffers (1, &vbo);
      glBindBuffer (GL_ARRAY_BUFFER, vbo);
      //glBufferData (GL_ARRAY_BUFFER, sizeof(glm::vec3)*8, cl_float3ToVec3(bodies[i].Points), GL_STATIC_DRAW);
      glBufferData (GL_ARRAY_BUFFER, sizeof(float)*24, bodies[i]->GetOBB(), GL_STATIC_DRAW);

      //color
      unsigned int colours_vbo = 0;
      glGenBuffers (1, &colours_vbo);
      glBindBuffer (GL_ARRAY_BUFFER, colours_vbo);
      glBufferData (GL_ARRAY_BUFFER, sizeof (glm::vec3)*8,  colors, GL_STATIC_DRAW);


      vao = 0;
      glGenVertexArrays (1, &vao);
      glBindVertexArray (vao);
      glBindBuffer (GL_ARRAY_BUFFER, vbo);
      glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 0, (GLubyte*)NULL);

      //color
      glBindBuffer (GL_ARRAY_BUFFER, colours_vbo);
      glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 0, NULL);

      glEnableVertexAttribArray (0);
      glEnableVertexAttribArray (1);

      //glColorPointer( 3, GL_FLOAT, sizeof(glm::vec3)*8, (*i).getColors() );
      //glVertexPointer( 3, GL_FLOAT, sizeof(glm::vec3)*8, (*i).getPoints() );
      glColorPointer( 3, GL_FLOAT_VEC3, 0, colors );
      //glVertexPointer( 3, GL_FLOAT_VEC3, 0, cl_float3ToVec3(bodies[i].Points));
      glVertexPointer( 3, GL_FLOAT, 0, bodies[i]->GetOBB());
      glDrawElements( GL_QUADS, 24, GL_UNSIGNED_INT, indices );

      glDisableClientState( GL_COLOR_ARRAY );
      glDisableClientState( GL_VERTEX_ARRAY );
  }

  glfwPollEvents ();
  glfwSwapBuffers (window);

  if(glfwGetKey(window,GLFW_KEY_M))
    setWireframe(true);
  if(glfwGetKey(window,GLFW_KEY_N))
    setWireframe(false);
  return 0;
}

int Renderer::Close()
{
  glfwTerminate();
  return 0;
}

GLFWwindow* Renderer::getWindow()
{
  return window;
}

void Renderer::setWireframe(bool wire)
{
  if(wire)
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    else
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}

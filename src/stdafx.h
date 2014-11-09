#ifndef STDAFX_H
#define STDAFX_H

#define ONE_DEG_IN_RAD (float)(M_PI/180.0) // 0.017444444
#define WIDTH (640)
#define HEIGHT (480)
#define numBoxes 2

#define AIRDRAG 0.98

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#define GLEW_STATIC
#include "GL/glew.h"
#include "GL/glfw3.h"
#include "CL/cl.h"
#define MEM_SIZE (9)
#define MAX_SOURCE_SIZE (0x100000)
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <glm/gtc/type_ptr.hpp>
#include "glm/gtc/quaternion.hpp"
#include "glm/gtx/quaternion.hpp"
//#include "tinythread.h"

#include "body.h"
#include "bodyInst.h"
#include "box.h"
//#include "sphere.h"
#include "camera.h"
#include "solver.h"
#include "renderer.h"



#endif // STDAFX_H

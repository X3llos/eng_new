#include "stdafx.h"

int main()
{
  std::vector<Body*> boxes2;
  for(int i=0; i<numBoxes;++i)
    {
    boxes2.push_back(new Box());
    }

  boxes2[0]->SetCenter(0.0f,-0.5f,0.0f);
  boxes2[1]->SetCenter(0.0f,-10.5f,0.0f);
  boxes2[0]->SetLengths(10.0f,0.5f,10.0f);
  boxes2[0]->SetVelocity(0.0f,0.0f,0.0f);
  boxes2[0]->isActive = true;
  boxes2[0]->SetAngularVelocity(0.0f,0.0f,0.0f);

  for(int i=1;i< numBoxes;i++)
    {
    boxes2[i]->SetCenter(0.0f,i*5.0f,0.0f);
    boxes2[i]->SetLengths(0.5f,0.5f,0.5f);
    boxes2[i]->SetVelocity(0.0f,0.0f,0.0f);
    boxes2[i]->isActive = true;
    if(i%2 == 0)
      {
        boxes2[i]->SetAngularVelocity(0.0f,0.0f,0.0f);
        boxes2[i]->SetCenter(0.0f,i*5.0f,0.0f);
        boxes2[i]->SetVelocity(0.0f,0.0f,0.0f);
      }
    else
      {
        boxes2[i]->AddAngularForce(0.0f,0.0f,0.0f);
        boxes2[i]->SetAngularVelocity(AngleToRad(0.0f),AngleToRad(0.0f),AngleToRad(40.0f));
      }
    boxes2[i]->SetWeight(2.0f);
    }
  Renderer::GetInstance().Init();
  Solver::GetInstance().Init();
  double previous_seconds = 0;

  {
  double current_seconds = glfwGetTime();
  double elapsed_seconds = current_seconds - previous_seconds;
  previous_seconds = current_seconds;
  Solver::GetInstance().UpdateCPU(elapsed_seconds, boxes2, 1);
  boxes2[0]->isActive = false;
  }
  bool runned = false;
  while (!glfwWindowShouldClose (Renderer::GetInstance().getWindow()))
  {
      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_P))
        runned = true;
      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_O))
        runned = false;
      if (glfwGetKey(Renderer::GetInstance().getWindow(), GLFW_KEY_F))
        {boxes2[2]->AddForce(2.0f,0.0f,0.0f);boxes2[2]->SetWeight(1.0f);}
      double current_seconds = glfwGetTime();
      double elapsed_seconds = current_seconds - previous_seconds;
      previous_seconds = current_seconds;
      if(runned)
        Solver::GetInstance().UpdateCPU(elapsed_seconds, boxes2, 0);
      Renderer::GetInstance().Update(elapsed_seconds, boxes2);
  }
  for(int i=numBoxes-1;i>=0;--i)
    {
      delete(boxes2[i]);
    }
  Renderer::GetInstance().Close();
  return 0;
}


#include "stdafx.h"

int main2() //GPU
{
  myBody boxes2[numBoxes];

  BodyInst::GetInstance().SetCenter(0.0f,0.0f,0.0f,&boxes2[0]);
  BodyInst::GetInstance().SetCenter(0.0f,-10.5f,0.0f,&boxes2[0]);
  BodyInst::GetInstance().SetLengths(10.0f,0.5f,10.0f,&boxes2[0]);
  BodyInst::GetInstance().SetVelocity(0.0f,0.0f,0.0f,&boxes2[0]);
  boxes2[0].isActive = 1;
  boxes2[0].type = 0;
  BodyInst::GetInstance().SetAngularVelocity(0.0f,0.0f,0.0f,&boxes2[0]);

  for(int i=1;i< numBoxes;i++)
    {
    BodyInst::GetInstance().SetCenter(2.0f,i*5.0f,0.0f,&boxes2[i]);
    BodyInst::GetInstance().SetLengths(0.5f,0.5f,0.5f,&boxes2[i]);
    BodyInst::GetInstance().SetVelocity(0.0f,0.0f,0.0f,&boxes2[i]);
    boxes2[i].isActive = 1;
    boxes2[i].type = 0;
    if(i%2 == 0)
      {
        BodyInst::GetInstance().SetAngularVelocity(0.0f,0.0f,0.0f,&boxes2[i]);
        BodyInst::GetInstance().SetCenter(0.0f,i*4.0f,0.0f,&boxes2[i]);
        BodyInst::GetInstance().SetVelocity(10.0f,0.0f,0.0f,&boxes2[i]);
      }
    else
      {
        BodyInst::GetInstance().AddAngularForce(0.0f,0.0f,0.0f,&boxes2[i]);
        BodyInst::GetInstance().SetAngularVelocity(AngleToRad(0.0f),AngleToRad(0.0f),AngleToRad(0.0f),&boxes2[i]);
      }
    boxes2[i].weight = 2.0;
    }
  Renderer::GetInstance().Init();
  Solver::GetInstance().Init();
  double previous_seconds = 0;

  {
  double current_seconds = glfwGetTime();
  double elapsed_seconds = current_seconds - previous_seconds;
  previous_seconds = current_seconds;
  Solver::GetInstance().UpdateGPU(elapsed_seconds, boxes2, 1);
  boxes2[0].isActive = 0;
  }
  bool runned = false;

  double nbFrames = 0;
  double lastTime = 0.0;
  while (!glfwWindowShouldClose (Renderer::GetInstance().getWindow()))
  {
      double currentTime = glfwGetTime();
           nbFrames++;
           if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1 sec ago
               // printf and reset timer
               printf("%f ms/frame\n", 1000.0/double(nbFrames));
               nbFrames = 0;
               lastTime += 1.0;
           }

      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_P))
        runned = true;
      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_O))
        runned = false;
      if (glfwGetKey(Renderer::GetInstance().getWindow(), GLFW_KEY_F))
        {BodyInst::GetInstance().AddForce(2.0f,0.0f,0.0f,&boxes2[1]);}
      double current_seconds = glfwGetTime();
      double elapsed_seconds = current_seconds - previous_seconds;
      previous_seconds = current_seconds;
      if(runned)
        Solver::GetInstance().UpdateGPU(elapsed_seconds, boxes2, 0);
      Renderer::GetInstance().UpdateGPU(elapsed_seconds, boxes2);
  }
  Renderer::GetInstance().Close();
  delete[] boxes2;
  return 0;
}


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
    boxes2[i]->SetCenter(2.0f,i*2.0f,0.0f);
    boxes2[i]->SetLengths(0.5f,0.5f,0.5f);
    boxes2[i]->SetVelocity(0.0f,0.0f,0.0f);
    boxes2[i]->isActive = true;
    if(i%3 == 0)
      {
        boxes2[i]->SetAngularVelocity(0.0f,0.0f,0.0f);
        boxes2[i]->SetCenter(3.0f,i*2.0f,-3.0f);
        boxes2[i]->SetVelocity(0.0f,0.0f,0.0f);
      }
    else
      {
        boxes2[i]->AddAngularForce(0.0f,0.0f,0.0f);
        boxes2[i]->SetAngularVelocity(AngleToRad(0.0f),AngleToRad(0.0f),AngleToRad(0.0f));
      }
    boxes2[i]->SetWeight(2.0f);
    }
  for(int i=1;i< numBoxes;i++)
    {
      switch(i%10)
        {
        case 0: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 1: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 2: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 3: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 4: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 5: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 6: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 7: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 8: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        case 9: boxes2[i]->SetCenter(-10.0f +(i%10)*2.0f,(i/10)*2.0f,-10.0f +(i%10)*2.0f); break;
        }
    }
  Renderer::GetInstance().Init();
  Solver::GetInstance().Init();
  double previous_seconds = 0;

  {
  double current_seconds = glfwGetTime();
  double elapsed_seconds = current_seconds - previous_seconds;
  previous_seconds = current_seconds;
  //Solver::GetInstance().UpdateCPU(elapsed_seconds, boxes2, 1);
  boxes2[0]->isActive = false;
  }
  bool runned = false;
  double nbFrames = 0;
  double lastTime = 0.0;
  while (!glfwWindowShouldClose (Renderer::GetInstance().getWindow()))
  {
      double currentTime = glfwGetTime();
           nbFrames++;
           if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1 sec ago
               // printf and reset timer
               printf("%f ms/frame\n", 1000.0/double(nbFrames));
               nbFrames = 0;
               lastTime += 1.0;
           }

      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_P))
        runned = true;
      if (glfwGetKey ( Renderer::GetInstance().getWindow(), GLFW_KEY_O))
        runned = false;
      if (glfwGetKey(Renderer::GetInstance().getWindow(), GLFW_KEY_F))
        {boxes2[1]->AddForce(2.0f,0.0f,0.0f);}
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


// controlDLL.cpp : Defines the entry point for the DLL application.

#include "servo.h"
#include "param.h"
#include "control.h"
// #include "UiAgent.h"
#include "PrVector.h"
#include "PrMatrix.h"
#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.
#include <math.h>
#include <algorithm>
using std::max;
using std::min;

struct CubicSpline {
double t0 , tf;
PrVector a0 , a1 , a2 , a3;
};
CubicSpline spline ;

CubicSpline CS;

// Compute total trajectory length
double computeTf ( GlobalVariables & gv)
{   
    //desired change in joint angle
    double delta_q1=gv.qd[0]-gv.q[0];
    double delta_q2=gv.qd[1]-gv.q[1];
    double delta_q3=gv.qd[2]-gv.q[2];

    //absolut change of the angles
    if(delta_q1<0.0)delta_q1=-delta_q1;
    if(delta_q2<0.0)delta_q2=-delta_q2;
    if(delta_q3<0.0)delta_q3=-delta_q3;
  

    //taking into consideration the stricter constraints 
    //max(tf_vel_constraint, tf_acc_coinstraint)
    //for each joint
    double min_tf_q1 = max((1.5*delta_q1)/gv.dqmax[0],sqrt((6*delta_q1)/gv.ddqmax[0]));

    double min_tf_q2 = max((1.5*delta_q2)/gv.dqmax[1],sqrt((6*delta_q2)/gv.ddqmax[1]));

    double min_tf_q3 = max((1.5*delta_q3)/gv.dqmax[2],sqrt((6*delta_q3)/gv.ddqmax[2]));
    
    //get longest necessary tf from the 3 joint angles
    double min_tf_q123=max(min_tf_q1,min_tf_q2); 
    min_tf_q123=max(min_tf_q123,min_tf_q3); 

    
    double cur= gv.curTime;
    

//adding 5% safety margin
    return cur + min_tf_q123*1.05;
}


void PrintDebug(GlobalVariables &gv);

// *******************************************************************
// Initialization functions
// *******************************************************************

void InitControl(GlobalVariables &gv)
{
  // This code runs before the first servo loop
}

void PreprocessControl(GlobalVariables &gv)
{
  // This code runs on every servo loop, just before the control law

  if ((gv.dof == 3) || (gv.dof == 6))
  {
    // get the correct joint angles depending on the current mode:
    double q1, q2, q3;
    if (gv.dof == 3)
    {
      q1 = gv.q[0];
      q2 = gv.q[1];
      q3 = gv.q[2];
    }
    else if (gv.dof == 6)
    {
      q1 = gv.q[1];
      q2 = gv.q[2];
      q3 = gv.q[4];
    }

    // Variable that holds the torque exerted by gravity for each joint
    
    PrVector3 g123 = PrVector3(0, 0, 0);

    // Compute g123 here!

    double c1 = cos(q1);
    double c12 = cos(q1 + q2);
    double c123 = cos(q1 + q2 + q3);
    double s1 = sin(q1);
    double s12 = sin(q1 + q2);
    double s123 = sin(q1 + q2 + q3);

    double r1 = R2;
    double r2 = 0.189738;
    double r3 = R6;
    double l1 = L2;
    double l2 = L3;
    double l3 = L6;
    double m1 = M2;
    double m2 = M3 + M4 + M5;
    double m3 = M6;
    double g = -9.81;

    g123[0] = g * (m3 * (l1 * c1 + l2 * s12 + r3 * s123) + (l1 * c1 + r2 * s12) * m2 + r1 * c1 * m1);
    g123[1] = g * (r2 * s12 * m2 + (l2 * s12 + r3 *s123) * m3);
    g123[2] = g* r3 * s123  * m3;

    // maps the torques to the right joint indices depending on the current mode:
    if (gv.dof == 3)
    {
      gv.G[0] = g123[0];
      gv.G[1] = g123[1];
      gv.G[2] = g123[2];
    }
    else if (gv.dof == 6)
    {
      gv.G[1] = g123[0];
      gv.G[2] = g123[1];
      gv.G[4] = g123[2];
    }
    // printing example, do not leave print inthe handed in solution
    // printVariable(g123, "g123");
  }
  else
  {
    gv.G = PrVector(gv.G.size());
  }
}

void PostprocessControl(GlobalVariables &gv)
{
  // This code runs on every servo loop, just after the control law
}

void initFloatControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initOpenControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNjholdControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initJholdControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNjmoveControl(GlobalVariables &gv)
{
      // Control Initialization Code Here
}

void initJmoveControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNjgotoControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initJgotoControl(GlobalVariables &gv)
{
   // Control Initialization Code Here
}

void initNjtrackControl(GlobalVariables &gv)
{ 
  auto q1 = gv.q[0];
  auto q2 = gv.q[1];
  auto q3 = gv.q[2];
  auto qd1 = gv.qd[0];
  auto qd2 = gv.qd[1];
  auto qd3 = gv.qd[2];
  auto dq1 = gv.dq[0];
  auto dq2 = gv.dq[1];
  auto dq3 = gv.dq[2];

  auto t0 =gv.curTime;
  auto tf = computeTf(gv);
  auto dt= tf-t0;

  CS.t0=t0;
  CS.tf=tf;
  CS.a0=PrVector3(q1, q2, q3);
  CS.a1=PrVector3(dq1, dq2, dq3);
  CS.a2=PrVector3(3/(dt*dt)*(qd1-q1), 3/(dt*dt)*(qd2-q2), 3/(dt*dt)*(qd3-q3));
  CS.a3=PrVector3(-2/(dt*dt*dt)*(qd1-q1), -2/(dt*dt*dt)*(qd2-q2), -2/(dt*dt*dt)*(qd3-q3));
}

void initJtrackControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNxtrackControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initXtrackControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNholdControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initHoldControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNgotoControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initGotoControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initNtrackControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initTrackControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initPfmoveControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initLineControl(GlobalVariables &gv)
{
  // Control Initialization Code Here
}

void initProj1Control(GlobalVariables &gv)
{
    gv.qd[0]=0.096;
    gv.qd[1]=0.967;
    gv.qd[2]=-1.061;

    

    gv.xd[0]=0;
    gv.xd[1]=0;
    gv.xd[2]=0;
    initNjtrackControl(gv);
}

void initProj2Control(GlobalVariables &gv)
{
  //CS is used as a global variable to store the starting time point
  CS.t0=gv.curTime;
}

void initProj3Control(GlobalVariables &gv)
{
  //CS is used as a global variable to store the starting time point
  CS.t0=gv.curTime;
}

// *******************************************************************
// Control laws
// *******************************************************************

void noControl(GlobalVariables &gv)
{
}

void floatControl(GlobalVariables &gv)
{
  PreprocessControl(gv);
  if (gv.dof == 3)
  {
    gv.tau[0] = gv.G[0];
    gv.tau[1] = gv.G[1];
    gv.tau[2] = gv.G[2];
  }
  else if (gv.dof == 6)
  {
    gv.tau[1] = gv.G[1];
    gv.tau[2] = gv.G[2];
    gv.tau[4] = gv.G[4];
  }

  // gv.tau = ?
  // this only works on the real robot unless the function is changed to use cout
  // the handed in solution must not contain any printouts
  // PrintDebug(gv);
}

void openControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void njholdControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void jholdControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void njmoveControl(GlobalVariables &gv)
{
  // float q_desired=0;
  //check if either 3 or 6 dof and 
  //chose the corresponded indexes
  if(gv.dof == 3 || gv.dof == 6)
  {
      if (gv.dof == 3)
      {
          // formulas for proportional control
          gv.tau[0] = -gv.kp[0] * (gv.q[0] - gv.qd[0]);
          gv.tau[1] = -gv.kp[1] * (gv.q[1] - gv.qd[1]);
          gv.tau[2] = -gv.kp[2] * (gv.q[2] - gv.qd[2]);
      }
      else if (gv.dof == 6)
      {
          gv.tau[1] = - gv.kp[1] * (gv.q[1] - gv.qd[1]);
          gv.tau[2] = - gv.kp[2] * (gv.q[2] - gv.qd[2]);
          gv.tau[4] = - gv.kp[4] * (gv.q[4] - gv.qd[4]);
      } 
  }


  
}

void jmoveControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void njgotoControl(GlobalVariables &gv)
{
  //check if either 3 or 6 dof and 
  //chose the corresponded indexes
  if(gv.dof == 3 || gv.dof == 6)
  {
    PreprocessControl(gv);
      if (gv.dof == 3)
      {
          gv.tau[0] = - gv.kp[0] * (gv.q[0] - gv.qd[0]) + gv.G[0];
          gv.tau[1] = - gv.kp[1] * (gv.q[1] - gv.qd[1]) + gv.G[1];
          gv.tau[2] = - gv.kp[2] * (gv.q[2] - gv.qd[2]) + gv.G[2];
      }
      else if (gv.dof == 6)
      {
          gv.tau[1] = -gv.kp[1] * (gv.q[1] - gv.qd[1]) + gv.G[1];
          gv.tau[2] = -gv.kp[2] * (gv.q[2] - gv.qd[2]) + gv.G[2];
          gv.tau[4] = -gv.kp[4] * (gv.q[4] - gv.qd[4]) + gv.G[4];
      } 
  }
}

void jgotoControl(GlobalVariables &gv)
{
   if(gv.dof == 3 || gv.dof == 6)
   {
    PreprocessControl(gv);
      if (gv.dof == 3)
      {
          gv.tau[0] = -gv.kp[0] * (gv.q[0] - gv.qd[0]) - gv.kv[0] * gv.dq[0] + gv.G[0];
          gv.tau[1] = -gv.kp[1] * (gv.q[1] - gv.qd[1]) - gv.kv[1] * gv.dq[1] + gv.G[1];
          gv.tau[2] = -gv.kp[2] * (gv.q[2] - gv.qd[2]) - gv.kv[2] * gv.dq[2] + gv.G[2];
      }
      else if (gv.dof == 6)
      {
          gv.tau[1] = -gv.kp[1] * (gv.q[1] - gv.qd[1]) - gv.kv[1] * gv.dq[1] + gv.G[1];
          gv.tau[2] = -gv.kp[2] * (gv.q[2] - gv.qd[2]) - gv.kv[2] * gv.dq[2] + gv.G[2];
          gv.tau[4] = -gv.kp[4] * (gv.q[4] - gv.qd[4]) - gv.kv[4] * gv.dq[4] + gv.G[4];
      } 
  }
}

void njtrackControl(GlobalVariables &gv)
{
  if(gv.curTime<=CS.tf)
  {
      auto a0 = CS.a0;
      auto a1 = CS.a1;
      auto a2 = CS.a2;
      auto a3 = CS.a3;
      auto t= gv.curTime;
      auto t0= CS.t0;
      auto dt = t-t0;

      auto qd1= a0[0] + a1[0]*dt + a2[0]*dt*dt + a3[0]*dt*dt*dt;
      auto qd2= a0[1] + a1[1]*dt + a2[1]*dt*dt + a3[1]*dt*dt*dt;
      auto qd3= a0[2] + a1[2]*dt + a2[2]*dt*dt + a3[2]*dt*dt*dt;
      auto dqd1= a1[0] + 2*a2[0]*dt + 3*a3[0]*dt*dt;
      auto dqd2= a1[1] + 2*a2[1]*dt + 3*a3[1]*dt*dt;
      auto dqd3= a1[2] + 2*a2[2]*dt + 3*a3[2]*dt*dt;

     gv.qd[0]=qd1;
     gv.qd[1]=qd2;
     gv.qd[2]=qd3;

     //saving acceleration of the spline
     //gv.xd[0]= 2*a2[0] + 3*a3[0]*dt;
     //gv.xd[1]= 2*a2[1] + 3*a3[1]*dt;
     //gv.xd[2]= 2*a2[2] + 3*a3[2]*dt;

      gv.tau[0]=-gv.kp[0]*(gv.q[0]-qd1) - gv.kv[0]*(gv.dq[0]-dqd1) + gv.G[0];
      gv.tau[1]=-gv.kp[1]*(gv.q[1]-qd2) - gv.kv[1]*(gv.dq[1]-dqd2) + gv.G[1];
      gv.tau[2]=-gv.kp[2]*(gv.q[2]-qd3) - gv.kv[2]*(gv.dq[2]-dqd3) + gv.G[2];
  }
  else
  {
      floatControl(gv); // Remove this line when you implement this controller
  }



}

void jtrackControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void nxtrackControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void xtrackControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void nholdControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void holdControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void ngotoControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void gotoControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void ntrackControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void trackControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void pfmoveControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void lineControl(GlobalVariables &gv)
{
  floatControl(gv); // Remove this line when you implement this controller
}

void proj1Control(GlobalVariables &gv)
{
   njtrackControl(gv);
}


void proj2Control(GlobalVariables &gv)
{
    //trajectory generation of a circle in x,y coordinates
    auto t = gv.curTime;
    //radius of the trajectory circle 
    auto r = 0.2;
    //center point coordinates 
    auto xc = 0.6;
    auto yc = 0.35;

    //initialization of desired trajectory
    double x_traj=0.0;
    double y_traj=0.0;
    double dx_traj=0.0;
    double dy_traj=0.0;

    auto dt = t-CS.t0;


    x_traj=xc + r*cos((2*M_PI)/5*dt);
    y_traj=yc + r*sin((2*M_PI)/5*dt);
    dx_traj= -((2*M_PI)/5)*r*sin((2*M_PI)/5*dt);
    dy_traj= ((2*M_PI)/5)*r*cos((2*M_PI)/5*dt);

    
     
    gv.xd[0]=x_traj;
    gv.xd[1]=y_traj;
    gv.xd[2]=0;


    double alpha_traj=0;
    double dalpha_traj=0;
    

    double F0=-gv.kp[0]*(gv.x[0]-x_traj) - gv.kv[0]*(gv.dx[0]-dx_traj); 
    double F1=-gv.kp[1]*(gv.x[1]-y_traj) - gv.kv[1]*(gv.dx[1]-dy_traj); 
    double F2=-gv.kp[2]*(gv.x[2]-alpha_traj) - gv.kv[2]*(gv.dx[2]-dalpha_traj);

    for(int i=0; i<3; i++)
    {
      gv.tau[i] = gv.Jtranspose[i][0]*F0 + gv.Jtranspose[i][1]*F1 + gv.Jtranspose[i][2]*F2;
    }
}

void proj3Control(GlobalVariables &gv)
{
  //trajectory generation of a circle in x,y coordinates
    auto t = gv.curTime;
    //radius of the trajectory circle 
    auto r = 0.2;
    //center point coordinates 
    auto xc = 0.6;
    auto yc = 0.35;

    //initialization of desired trajectory and desired trajectory velovcity
    double x_traj=0.0;
    double y_traj=0.0;
    double dx_traj=0.0;
    double dy_traj=0.0;

    //get current time in relation to start time 
    auto dt = t-CS.t0;
    //movement trajectory during the 3 revolutions during the 3 phases 3 with the parabolic blends
    if(dt<=20)
    {
        //the in order to allow a parabolic blend of the
        //speed the circular frequency is time dependend

        double beta_dot;
        double beta;
        //linear acceleration phase
        if(dt<5)
        {
            beta_dot=dt*(2*M_PI)/25;
            beta=0.5*beta_dot*dt;
        }
        //after reaching the desired velocity 3pi/5s at t=5s the 
        //circular velocity can be held constant
        else if (dt>=5 && dt <15)
        {
            beta_dot=(2*M_PI)/5; 
            beta=beta_dot*dt-M_PI;
        }
        
        //as at timepoint 15s the velocity needs to be 0
        //the linear deceleration phase needs to start at t=10s
        else if (dt>=15 && dt<20)
        {
          //the dt reverse is just a mathematical trick that allows the cos/sin to continue
          //in the correct direction and not to reset
          dt=dt-20;
          beta_dot=-(2*M_PI)/25*dt;
          beta=0.5*beta_dot*dt+6*M_PI;
        }
        
        

        x_traj=xc + r*cos(beta);
        y_traj=yc + r*sin(beta);
        dx_traj= -beta_dot*r*sin(beta);
        dy_traj= beta_dot*r*cos(beta);

        //to plot the beta/beta_dot
        //parallel to the resulting desired x and y trajectory  
        gv.xd[0]=beta;
        gv.xd[1]=beta_dot;
        gv.xd[2]=0;

        gv.qd[0]=x_traj;
        gv.qd[1]=y_traj;
        


        double alpha_traj=0;
        double dalpha_traj=0;
        

        double F0=-gv.kp[0]*(gv.x[0]-x_traj) - gv.kv[0]*(gv.dx[0]-dx_traj); 
        double F1=-gv.kp[1]*(gv.x[1]-y_traj) - gv.kv[1]*(gv.dx[1]-dy_traj); 
        double F2=-gv.kp[2]*(gv.x[2]-alpha_traj) - gv.kv[2]*(gv.dx[2]-dalpha_traj);

        for(int i=0; i<3; i++)
        {
          gv.tau[i] = gv.Jtranspose[i][0]*F0 + gv.Jtranspose[i][1]*F1 + gv.Jtranspose[i][2]*F2;
        }
    }
    //after 3 revolutions tranverse to flow control
    else
    {
         floatControl(gv);
    }

    
}

// *******************************************************************
// Debug function
// *******************************************************************

void PrintDebug(GlobalVariables &gv)
{
  // Replace this code with any debug information you'd like to get
  // when you type "pdebug" at the prompt.
  printf("This sample code prints the torque and mass\n");
  gv.tau.display("tau");
  gv.A.display("A");
}

#ifdef WIN32
// *******************************************************************
// XPrintf(): Replacement for printf() which calls ui->VDisplay()
// whenever the ui object is available.  See utility/XPrintf.h.
// *******************************************************************

int XPrintf(const char *fmt, ...)
{
  int returnValue;
  va_list argptr;
  va_start(argptr, fmt);

  returnValue = vprintf(fmt, argptr);

  va_end(argptr);
  return returnValue;
}
#endif // #ifdef WIN32

/********************************************************

END OF DEFAULT STUDENT FILE

ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS

*******************************************************/

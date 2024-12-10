/*

 Please fill in the function stubs provided in this file

 For achieving full points on this exercise, explain the implemented equations
 in the accompanying pdf. Especially:
   - which parameters were used for computation?
   - were equations simplified? What was the original equation?
   - when introducing additional variables, what do they contain?


 * Do not use additional includes
 * Do not include/link additional files
 * Do not change the predefined function signature
 * Do not change the header file

 Tip: use the main() function to test your implementation.

*/

#include "forwardkinematics.hpp" //Have a look at this header file, it declares the class ForwardKinematicsPuma2D
#include <cmath>                 //use sin, cos from cmath for your computation
#include <iostream>
using namespace std;


#ifdef UNITTEST
    #define main STUDENTS_MAIN
#endif


/*
Convenience function to print out a homogenous transform
*/
void print_HTransform(HTransform tf)
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\n---------------------------\n";

    cout << tf[0][0]<<"  "<<tf[0][1]<<"  "<<tf[0][2]<<"  "<<tf[0][3]<<endl;
    cout << tf[1][0]<<"  "<<tf[1][1]<<"  "<<tf[1][2]<<"  "<<tf[1][3]<<endl;
    cout << tf[2][0]<<"  "<<tf[2][1]<<"  "<<tf[2][2]<<"  "<<tf[2][3]<<endl;
    cout << tf[3][0]<<"  "<<tf[3][1]<<"  "<<tf[3][2]<<"  "<<tf[3][3]<<endl;
    cout <<   "---------------------------\n";
}

/*
Convenience function to print out a 3x3 Jacobian matrix
*/
void print_Jacobian(float Jacobian[3][3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\n+++++++++++++++++++++++++++\n";
    cout << Jacobian[0][0]<<"  "<<Jacobian[0][1]<<"  "<<Jacobian[0][2]<<endl;
    cout << Jacobian[1][0]<<"  "<<Jacobian[1][1]<<"  "<<Jacobian[1][2]<<endl;
    cout << Jacobian[2][0]<<"  "<<Jacobian[2][1]<<"  "<<Jacobian[2][2]<<endl;
    cout <<   "+++++++++++++++++++++++++++\n";
}


/*
Convenience function to print out a position
*/
void print_Position(float F[3])
{
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(3);
    cout << "\nooooooooooooooooooooooooooo\n";
    cout << F[0]<<"  "<<F[1]<<"  "<< F[2]<<endl;
    cout <<   "ooooooooooooooooooooooooooo\n";
}


/*
already implemented

Set the robot's joint values and recompute the forward kinematics.

a1, a2 and a3 are assumed to be given in radians!
*/
void ForwardKinematicsPuma2D::setJoints(float a1, float a2, float a3)
{
    //store joint angles
    angles[0] = a1;
    angles[1] = a2;
    angles[2] = a3;
    
    //recompute the dependent variables
    computeDH();
    computeT0_1();
    computeT1_2();
    computeT2_3();
    computeT3_E();
    computeT0_E();
    computeF();
    computeJ();
}




/***********************************************/
/******************EDIT BELOW ******************/
/***********************************************/

/*
updates the variable T0_1

<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeT0_1()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3
   //row in DH
   int i=0;

   //T01 matrix row one
   T0_1[0][0] = cos(DH[i][3]);
   T0_1[0][1] = -sin(DH[i][3]);
   T0_1[0][2] = 0.0;
   T0_1[0][3] = DH[i][0];

   //T01 matrix row two
   T0_1[1][0] = sin(DH[i][3])*cos(DH[i][0]);
   T0_1[1][1] = cos(DH[i][3])*cos(DH[i][0]);
   T0_1[1][2] = -sin(DH[i][0]);
   T0_1[1][3] = -sin(DH[i][0])*DH[i][2];

   //T01 matrix row three
   T0_1[2][0] = sin(DH[i][3])*sin(DH[i][0]);
   T0_1[2][1] = cos(DH[i][3])*sin(DH[i][0]);
   T0_1[2][2] = cos(DH[i][0]);
   T0_1[2][3] = cos(DH[i][0])*DH[i][2];

   //T01 matrix row four
   T0_1[3][0] = 0.0;
   T0_1[3][1] = 0.0;
   T0_1[3][2] = 0.0;
   T0_1[3][3] = 1.0;
}


/*
updates the variable T1_2

<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeT1_2()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

    //row in DH
   int i=1;

   //T12 matrix row one
   T1_2[0][0] = cos(DH[i][3]);
   T1_2[0][1] = -sin(DH[i][3]);
   T1_2[0][2] = 0.0;
   T1_2[0][3] = DH[i][0];

   //T12 matrix row two
   T1_2[1][0] = sin(DH[i][3])*cos(DH[i][0]);
   T1_2[1][1] = cos(DH[i][3])*cos(DH[i][0]);
   T1_2[1][2] = -sin(DH[i][0]);
   T1_2[1][3] = -sin(DH[i][0])*DH[i][2];

   //T12 matrix row three
   T1_2[2][0] = sin(DH[i][3])*sin(DH[i][0]);
   T1_2[2][1] = cos(DH[i][3])*sin(DH[i][0]);
   T1_2[2][2] = cos(DH[i][0]);
   T1_2[2][3] = cos(DH[i][0])*DH[i][2];

   //T12 matrix row four
   T1_2[3][0] = 0.0;
   T1_2[3][1] = 0.0;
   T1_2[3][2] = 0.0;
   T1_2[3][3] = 1.0;
}

/*
updates the variable T2_3

<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeT2_3()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

    //row in DH
   int i=2;

   //T23 matrix row one
   T2_3[0][0] = cos(DH[i][3]);
   T2_3[0][1] = -sin(DH[i][3]);
   T2_3[0][2] = 0.0;
   T2_3[0][3] = DH[i][0];

   //T23 matrix row two
   T2_3[1][0] = sin(DH[i][3])*cos(DH[i][0]);
   T2_3[1][1] = cos(DH[i][3])*cos(DH[i][0]);
   T2_3[1][2] = -sin(DH[i][0]);
   T2_3[1][3] = -sin(DH[i][0])*DH[i][2];

   //T23 matrix row three
   T2_3[2][0] = sin(DH[i][3])*sin(DH[i][0]);
   T2_3[2][1] = cos(DH[i][3])*sin(DH[i][0]);
   T2_3[2][2] = cos(DH[i][0]);
   T2_3[2][3] = cos(DH[i][0])*DH[i][2];

   //T23 matrix row four
   T2_3[3][0] = 0.0;
   T2_3[3][1] = 0.0;
   T2_3[3][2] = 0.0;
   T2_3[3][3] = 1.0;
}


/*
updates the variable T3_E

<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeT3_E()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row in DH
   int i=3;

   //T3E matrix row one 
   T3_E[0][0] = cos(DH[i][3]);
   T3_E[0][1] = -sin(DH[i][3]);
   T3_E[0][2] = 0.0;
   T3_E[0][3] = DH[i][0];

   //T3E matrix row two  
   T3_E[1][0] = sin(DH[i][3])*cos(DH[i][0]);
   T3_E[1][1] = cos(DH[i][3])*cos(DH[i][0]);
   T3_E[1][2] = -sin(DH[i][0]);
   T3_E[1][3] = -sin(DH[i][0])*DH[i][2];

   //T3E matrix row three 
   T3_E[2][0] = sin(DH[i][3])*sin(DH[i][0]);
   T3_E[2][1] = cos(DH[i][3])*sin(DH[i][0]);
   T3_E[2][2] = cos(DH[i][0]);
   T3_E[2][3] = cos(DH[i][0])*DH[i][2];

   //T3E matrix row four 
   T3_E[3][0] = 0.0;
   T3_E[3][1] = 0.0;
   T3_E[3][2] = 0.0;
   T3_E[3][3] = 1.0;
}


/*

This function updates the variable T0_E

<ADD EXPLANATION OF CODE>

*/
void ForwardKinematicsPuma2D::computeT0_E()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3
   double c123 = cos(angles[0] + angles[1] + angles[2]);
   double c12= cos(angles[0] + angles[1]);
   double c1 = cos(angles[0]);
   double s123= sin(angles[0] + angles[1] + angles[2]);
   double s12= sin(angles[0] + angles[1]);
   double s1= sin(angles[0]);


   //T0E matrix row one
   T0_E[0][0] = s123;
   T0_E[0][1] = c123;
   T0_E[0][2] = 0.0;
   T0_E[0][3] = s123 * l3 + s12 * l2 + c1 * l1;

   //T0E matrix row two
   T0_E[1][0] = -c123;
   T0_E[1][1] = s123;
   T0_E[1][2] = 0.0;
   T0_E[1][3] = - c123 * l3 - c12 * l2 + s1 * l1;

   //T0E matrix row three
   T0_E[2][0] = 0.0;
   T0_E[2][1] = 0.0;
   T0_E[2][2] = 1.0;
   T0_E[2][3] = 0.0;

   //T0E matrix row four
   T0_E[3][0] = 0.0;
   T0_E[3][1] = 0.0;
   T0_E[3][2] = 0.0;
   T0_E[3][3] = 1.0;
}



/*
This function updates the variables ee_x, ee_y, ee_alpha

<ADD EXPLANATION OF CODE>

*/
void ForwardKinematicsPuma2D::computeF()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   F[0] = 0.0; //x
   F[1] = 0.0; //y
   F[2] = 0.0; //alpha

   F[0]=T0_E[0][3];
   F[1]=T0_E[1][3];
   F[2]=angles[0] +  angles[1] - 0.5*M_PI + angles[2];
}


/*
This function updates the variable J

<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeJ()
{

   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3
   double c123 = cos(angles[0] + angles[1] + angles[2]);
   double c12= cos(angles[0] + angles[1]);
   double c1 = cos(angles[0]);
   double s123= sin(angles[0] + angles[1] + angles[2]);
   double s12= sin(angles[0] + angles[1]);
   double s1= sin(angles[0]);


   //row 1
   J[0][0] = -s1*l1 + c12*l2 + c123*l3;
   J[0][1] = c12*l2 + c123*l3;
   J[0][2] = c123*l3;

   //row 2
   J[1][0] = c1*l1 + s12*l2 + s123*l3;
   J[1][1] =  s12*l2 + s123*l3;
   J[1][2] =  s123*l3;

   //row 3
   J[2][0] = 1.0;
   J[2][1] = 1.0;
   J[2][2] = 1.0;
}

/* This function computes the DH params
   and sets the internal member variable DH
<ADD EXPLANATION OF CODE>
*/
void ForwardKinematicsPuma2D::computeDH()
{



   // compute from
   // angles[0], angles[1], angles[2], l1, l2, and l3

   //row vector
   DH[0][0] = 0.0; 
   DH[0][1] = 0.0; 
   DH[0][2] = 0.0; 
   DH[0][3] = angles[0];

   //row vector
   DH[1][0] = 0.0;
   DH[1][1] = l1;
   DH[1][2] = 0.0;
   DH[1][3] = angles[1]-0.5*M_PI; 

   //row vector
   DH[2][0] = 0.0; 
   DH[2][1] = l2; 
   DH[2][2] = 0.0; 
   DH[2][3] = angles[2];

   //row vector
   DH[3][0] = 0.0;
   DH[3][1] = l3;
   DH[3][2] = 0.0;
   DH[3][3] = 0.0;
}


/*
Example code to test your functions:

You are free to change main() as you like
*/
int main()
{
 ForwardKinematicsPuma2D* fk = new ForwardKinematicsPuma2D();
 fk->setJoints(0.0,0.0,0.0); //example, try out different values
 cout << "********************Testing Transforms**************"<<endl;
 print_HTransform(fk->T0_1);
 print_HTransform(fk->T1_2);
 print_HTransform(fk->T2_3);
 print_HTransform(fk->T3_E);
 print_HTransform(fk->T0_E);
 cout << "********************Testing F***********************"<<endl;
 print_Position(fk->F);
 cout << "********************Testing J***********************"<<endl;
 print_Jacobian(fk->J);
 return 0;
}

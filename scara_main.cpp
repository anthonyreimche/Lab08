/*|SCARA Simulator|------------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara_main.c
#
# Description:
#   This program demonstrates intermediate control over the SCARA Robot 
# Simulator. The simulator moves with only commands to control the motor 
# angles. To move to a desired (x, y) coordinate requires the implementation of
# inverse kinematics. Intermediate control introduces linear interpolation.
# Available commands are listed below.
#
# Other Programs:
#   ScaraRobotSimulator.exe (Version 4.3)
#
# Simulator Commands:
#  - PEN_UP
#  - PEN_DOWN
#  - PEN_COLOR <r> <g> <b>
#  - CYCLE_PEN_COLORS ON/OFF
#  - ROTATE_JOINT ANG1 <deg1> ANG2 <deg2>
#  - CLEAR_TRACE
#  - CLEAR_REMOTE_COMMAND_LOG
#  - CLEAR_POSITION_LOG
#  - SHUTDOWN_SIMULATION
#  - MOTOR_SPEED HIGH/MEDIUM/LOW
#  - MESSAGE <"string">
#  - HOME
#  - END
#
# Other Information:
#  - IP Address: 127.0.0.1 Port 1270
#  - BCIT Blue: 10 64 109
#  - If using VS Code, add the following args to tasks.json g++ build task.
#     "-std=c++11"
#		"-lwsock32"
#		"-Wno-deprecated"
#  - Also change the "${file}" argument to "scara_main.cpp" and add "scara.cpp".
#
# Author: Anthony Reimche
# Date Created: 04 11 2025
# Last Modified: 04 11 2025
# -----------------------------------------------------------------------------*/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>  // <list of functions used>
#include <math.h>   // <list of functions used>
#include "scara.h"  // <list of functions and structures used> // Your code should be inside here.

/*|Enumerations|---------------------------------------------------------------*/
typedef enum {
	JOINT = 1, LINEAR, QUIT
} menu;

/*|Global Variables|-----------------------------------------------------------*/
extern CRobot robot;

/*|Function Declarations|------------------------------------------------------*/
void moveScaraJTB();
void moveScaraLTB();
void setScaraAngles (double, double);
void setScaraPen(char);
void setScaraColor(int, int, int);


int main(){
   	// Customize Output
	system("COLOR 0A");
	system("CLS");

	// Variables
	int select = 0;

	// Initialize SCARA Simulator V3
	if(!robot.Initialize()) return 0;
	robot.Send("PEN_UP\n");
	robot.Send("HOME\n");
	robot.Send("CLEAR_TRACE\n");
	robot.Send("CLEAR_POSITION_LOG\n");
	robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");

	// Repeat Until 4: Quit is selected
	do {
		system("CLS");
		// Display the Menu
		printf("==========MAIN MENU==========\n\n");
		printf("1) 4.5 Joint Interpolated Move\n");
		printf("2) 4.7 Linear Interpolated Move\n");
		printf("3) Quit\n\n");

		// Get the User Input
		scanf("%d", &select);
		getchar();

		// Execute the Selected Program
		switch (select) {
		case JOINT:
			moveScaraJTB();
			break;
		case LINEAR:
			moveScaraLTB();
			break;
		case QUIT:
			robot.Send("END\n");
			robot.Close();
			break;
		default:
			printf("\n\nPlease Select one of the Menu Items.\n");
			getchar();
		}
	} while(select != QUIT);

	// Close the Connection to the SCARA Simulator V3
   robot.Send("END\n");
   printf("\n\nPress ENTER to end the program...\n");
   getchar();

   
   robot.Close(); // close remote connection
   return 0;
}

/****************************************************************************************
* Function: moveScaraJTB
*
* Description:
*	This function will test moveScaraJ.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: May 1, 2021 by Isaiah Regacho
*****************************************************************************************/
void moveScaraJTB() {
	// Starting Point
	SCARA_ROBOT robot = initScaraState(600, 0, LEFT_ARM_SOLUTION, {'u', {24, 0, 66}}, 'L');
	moveScaraJ(&robot);

	robot.armPos.x = 300;
	robot.armPos.y = 300;
	robot.armPos.armSol = RIGHT_ARM_SOLUTION;
	robot.toolPos.penPos = 'u';
	scaraIK(robot.armPos.x,robot.armPos.y,&robot.armPos.theta1,&robot.armPos.theta2,robot.armPos.armSol);
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

	robot.armPos.x = 300;
	robot.armPos.y = 0;
	robot.armPos.armSol = RIGHT_ARM_SOLUTION;
	robot.toolPos.penPos = 'd';
	scaraIK(robot.armPos.x,robot.armPos.y,&robot.armPos.theta1,&robot.armPos.theta2,robot.armPos.armSol);
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

	robot.armPos.x = 300;
	robot.armPos.y = -300;
	robot.armPos.armSol = LEFT_ARM_SOLUTION;
	robot.toolPos.penPos = 'd';
	scaraIK(robot.armPos.x,robot.armPos.y,&robot.armPos.theta1,&robot.armPos.theta2,robot.armPos.armSol);
	moveScaraJ(&robot);
	scaraDisplayState(robot);
	getchar();

}

/****************************************************************************************
* Function: moveScaraJTB
*
* Description:
*	This function will test moveScaraJ.
*
* Inputs: void
*
* Returns: void
*
* Last Modified: April 11, 2025, by Anthony Reimche
*****************************************************************************************/
void moveScaraLTB() {
	LINE_DATA lineData;

	// Initialized Simulator from this Point
	SCARA_ROBOT robot = initScaraState(300, 300, RIGHT_ARM_SOLUTION, {'d', {0, 0, 255}}, 'H');
	moveScaraJ(&robot);

	// Easy Set of Lines
	lineData = initLine(300, 0, 300, 300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(100, 500, 300, 300, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(300, 0, 300, -300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(100, -500, 300, -300, 20);
	moveScaraL(&robot, lineData);
	
	// Medium Set of Lines
	lineData = initLine(0, 600, 600, 0, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(600, 0, 0, -600, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(300, 300, -500, 300, 10);
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, 300, -300, 500, 5);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(300, -300, -500, -300, 10);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, -300, -300, -500, 5);
	moveScaraL(&robot, lineData);

	// Hard Set of Lines
	lineData = initLine(-500, 300, 600, 0, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(600, 0, -300, 500, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, -300, 600, 0, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(600, 0, -300, -500, 20);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, -300, -300, 500, 10);
	moveScaraL(&robot, lineData);
	
	lineData = initLine(-500, 300, -300, -500, 10);
	moveScaraL(&robot, lineData);

	// Challenge Set of Lines
	lineData = initLine(-500, 300, 0, -600, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(0, -600, -300, 500, 20); 
	moveScaraL(&robot, lineData);

	lineData = initLine(-500, -300, 0, 600, 20);
	moveScaraL(&robot, lineData);

	lineData = initLine(0, 600, -300, -500, 20);
	moveScaraL(&robot, lineData);
}


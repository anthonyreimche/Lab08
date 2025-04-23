/*|ROBT 1270: SCARA |----------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Intermediate Control
# Program: scara.cpp
#
# Description:
#   This program contains code for drawing lines with the SCARA.
#
# Author: <Your Name>
# Date Created: <Day Started>
# Last Modified: <Today's Date>
# -----------------------------------------------------------------------------*/

/*|Includes|-------------------------------------------------------------------*/
#include "scara.h"

/*|Global Variables|-----------------------------------------------------------*/
CRobot robot;     // the global robot Class instance.  Can be used everywhere
                  // robot.Initialize()
                  // robot.Send()
                  // robot.Close()

/*|Function Definitions|-------------------------------------------------------*/
/****************************************************************************************
* Function: scaraDisplayState
*
* Description:
*	Displays the SCARA information. Feel free to modify.
*
* Inputs:
*	scaraState  - Contains SCARA information
*
* Returns: void
*
* Last Modified: May 1, 2021 by Isaiah Regacho
*****************************************************************************************/
void scaraDisplayState(SCARA_ROBOT scaraState) {
	SCARA_POS arm = scaraState.armPos;
	SCARA_TOOL tool = scaraState.toolPos;

	printf("|SCARA STATE|\n");

	// Display Position
	printf("| Theta 1 | Theta 2 |    X    |    Y    |   Arm   |\n");
	printf("|%9.2lf|%9.2lf|%9.2lf|%9.2lf|    %d    |\n", arm.theta1, arm.theta2, arm.x, arm.y, arm.armSol);

	// Display Tool
	printf("|Position |   RED   |  GREEN  |   BLUE  |\n");
	printf("|    %c    |   %3d   |   %3d   |   %3d   |\n", tool.penPos, tool.penColor.r, tool.penColor.g, tool.penColor.b);
}

/****************************************************************************************
* Function: initScaraState
*
* Description:
*	Initializes the SCARA robot state with the given parameters.
*
* Inputs:
*	x        - X coordinate
*	y        - Y coordinate
*	armSol   - Arm solution (LEFT_ARM_SOLUTION or RIGHT_ARM_SOLUTION)
*	penState - Tool state (pen position and color)
*	mtrSpeed - Motor speed ('H', 'M', or 'L')
*
* Returns: SCARA_ROBOT - The initialized SCARA robot state
*
* Last Modified: April 11, 2025
*****************************************************************************************/
SCARA_ROBOT initScaraState(double x, double y, int armSol, SCARA_TOOL penState, char mtrSpeed) {
    SCARA_ROBOT scaraState;
    
    // Initialize position
    scaraState.armPos.x = x;
    scaraState.armPos.y = y;
    scaraState.armPos.armSol = armSol;
    
    // Calculate inverse kinematics to get joint angles
    scaraIK(x, y, &scaraState.armPos.theta1, &scaraState.armPos.theta2, armSol);
    
    // Initialize tool state
    scaraState.toolPos = penState;
    
    // Initialize motor speed
    scaraState.motorSpeed = mtrSpeed;
    
    return scaraState;
}

/****************************************************************************************
* Function: moveScaraJ
*
* Description:
*	Moves the SCARA robot using joint interpolation.
*
* Inputs:
*	scaraState - Pointer to the SCARA robot state
*
* Returns: int - 0 for success
*
* Last Modified: April 22, 2025
*****************************************************************************************/
int moveScaraJ(SCARA_ROBOT* scaraState) {
    // Use scaraSetState to update the robot state
    // This will only send commands for values that have changed
    scaraSetState(*scaraState);
    
    return 0;
}

/****************************************************************************************
* Function: initLine
*
* Description:
*	Initializes a line with start and end points and number of points.
*
* Inputs:
*	xA     - X coordinate of start point
*	yA     - Y coordinate of start point
*	xB     - X coordinate of end point
*	yB     - Y coordinate of end point
*	numPts - Number of points along the line
*
* Returns: LINE_DATA - The initialized line data
*
* Last Modified: April 11, 2025
*****************************************************************************************/
LINE_DATA initLine(double xA, double yA, double xB, double yB, int numPts) {
    LINE_DATA line;
    
    // Initialize line endpoints
    line.xA = xA;
    line.yA = yA;
    line.xB = xB;
    line.yB = yB;
    
    // Set number of points
    line.numPts = numPts;
    
    // Default color (can be modified later)
    line.color.r = 0;
    line.color.g = 0;
    line.color.b = 0;
    
    return line;
}

/****************************************************************************************
* Function: moveScaraL
*
* Description:
*	Moves the SCARA robot along a line using linear interpolation.
*   First moves to the line start point with pen up (if not already there),
*   then draws the line with the pen down.
*
* Inputs:
*	scaraState - Pointer to the SCARA robot state
*	line       - Line data for the movement
*
* Returns: int - 0 for success
*
* Last Modified: April 22, 2025
*****************************************************************************************/
int moveScaraL(SCARA_ROBOT *scaraState, LINE_DATA line) {
    // Check if we need to move to the start point first
    if (fabs(scaraState->armPos.x - line.xA) > POINT_TOL || 
        fabs(scaraState->armPos.y - line.yA) > POINT_TOL) {
        
        // First ensure the pen is up before any movement
        SCARA_ROBOT penUpState = *scaraState;
        penUpState.toolPos.penPos = 'u';
        scaraSetState(penUpState);
        
        // Create a temporary state for moving to the start point with pen up
        SCARA_ROBOT startPointState = *scaraState;
        startPointState.armPos.x = line.xA;
        startPointState.armPos.y = line.yA;
        startPointState.toolPos.penPos = 'u'; // Ensure pen up for moving to start
        
        // Calculate inverse kinematics for the start point
        scaraIK(line.xA, line.yA, &startPointState.armPos.theta1, 
                &startPointState.armPos.theta2, scaraState->armPos.armSol);
        
        // Move to the start point
        scaraSetState(startPointState);
        
        // Update the scaraState with the new position
        scaraState->armPos.x = line.xA;
        scaraState->armPos.y = line.yA;
        scaraState->armPos.theta1 = startPointState.armPos.theta1;
        scaraState->armPos.theta2 = startPointState.armPos.theta2;
        scaraState->toolPos.penPos = 'u'; // State reflects pen is up
    }
    
    // Set the pen down for drawing the line
    SCARA_ROBOT penDownState = *scaraState;
    penDownState.toolPos.penPos = 'd'; // Put pen down for drawing
    scaraSetState(penDownState);
    scaraState->toolPos.penPos = 'd'; // Update state to reflect pen is down
    
    // Memory allocation for variable sized array of SCARA_ROBOT
    SCARA_ROBOT *robline = (SCARA_ROBOT*) malloc(sizeof(SCARA_ROBOT) * line.numPts);
    
    // Calculate points along the line
    for (int i = 0; i < line.numPts; i++) {
        // Linear interpolation between start and end points
        double t = (double)i / (line.numPts - 1);  // Parameter from 0 to 1
        double x = line.xA + t * (line.xB - line.xA);
        double y = line.yA + t * (line.yB - line.yA);
        
        // Copy the current state for this point
        robline[i] = *scaraState;
        
        // Update only the position for this point
        robline[i].armPos.x = x;
        robline[i].armPos.y = y;
        
        // Calculate inverse kinematics for the new position
        scaraIK(x, y, &robline[i].armPos.theta1, &robline[i].armPos.theta2, scaraState->armPos.armSol);
        
        // Move to this point using scaraSetState
        scaraSetState(robline[i]);
    }
    
    // Update the scaraState to the end position
    scaraState->armPos.x = line.xB;
    scaraState->armPos.y = line.yB;
    scaraIK(line.xB, line.yB, &scaraState->armPos.theta1, &scaraState->armPos.theta2, scaraState->armPos.armSol);
    
    // Free up memory allocated
    free(robline);
    
    // Return success
    return 0;
}

/****************************************************************************************
* Function: scaraFK
*
* Description:
*	Calculates forward kinematics for the SCARA robot.
*
* Inputs:
*	ang1   - Angle of the first joint (degrees)
*	ang2   - Angle of the second joint (degrees)
*	toolX  - Pointer to store the calculated X coordinate
*	toolY  - Pointer to store the calculated Y coordinate
*
* Returns: int - 0 for success
*
* Last Modified: April 11, 2025
*****************************************************************************************/
int scaraFK(double ang1, double ang2, double* toolX, double* toolY) {
    // Convert angles from degrees to radians
    double theta1 = ang1 * PI / 180.0;
    double theta2 = ang2 * PI / 180.0;
    
    // Calculate end effector position using forward kinematics
    *toolX = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    *toolY = L1 * sin(theta1) + L2 * sin(theta1 + theta2);
    
    return 0;
}

/****************************************************************************************
* Function: scaraIK
*
* Description:
*	Calculates inverse kinematics for the SCARA robot.
*
* Inputs:
*	toolX  - X coordinate of the tool
*	toolY  - Y coordinate of the tool
*	ang1   - Pointer to store the calculated angle of the first joint (degrees)
*	ang2   - Pointer to store the calculated angle of the second joint (degrees)
*	arm    - Arm solution (LEFT_ARM_SOLUTION or RIGHT_ARM_SOLUTION)
*
* Returns: int - 0 for success, -1 for failure (unreachable position)
*
* Last Modified: April 11, 2025
*****************************************************************************************/
int scaraIK(double toolX, double toolY, double* ang1, double* ang2, int arm) {
    double r = sqrt(toolX * toolX + toolY * toolY);
    
    // Check if the position is reachable
    if (r > L1 + L2 || r < fabs(L1 - L2)) {
        // Position is unreachable
        return -1;
    }
    
    // Calculate angle 2 using cosine law
    double cos_theta2 = (toolX * toolX + toolY * toolY - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    
    // Clamp to valid range to avoid numerical issues
    if (cos_theta2 > 1.0) cos_theta2 = 1.0;
    if (cos_theta2 < -1.0) cos_theta2 = -1.0;
    
    // Calculate theta2 based on arm solution
    double theta2;
    if (arm == RIGHT_ARM_SOLUTION) {
        theta2 = -acos(cos_theta2);  // Elbow down (negative angle)
    } else {
        theta2 = acos(cos_theta2);   // Elbow up (positive angle)
    }
    
    // Calculate theta1
    double theta1 = atan2(toolY, toolX) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
    
    // Convert angles from radians to degrees
    *ang1 = theta1 * 180.0 / PI;
    *ang2 = theta2 * 180.0 / PI;
    
    return 0;
}

/****************************************************************************************
* Function: rotate
*
* Description:
*	Rotates the SCARA robot joints to the specified angles.
*
* Inputs:
*	ang1 - Angle of the first joint (degrees)
*	ang2 - Angle of the second joint (degrees)
*
* Returns: void
*
* Last Modified: April 11, 2025
*****************************************************************************************/
void rotate(double ang1, double ang2) {
    char command[MAX_STRING];
    sprintf(command, "ROTATE_JOINT ANG1 %.2f ANG2 %.2f\n", ang1, ang2);
    robot.Send(command);
}

/****************************************************************************************
* Function: setColor
*
* Description:
*	Sets the pen color for the SCARA robot.
*
* Inputs:
*	r - Red component (0-255)
*	g - Green component (0-255)
*	b - Blue component (0-255)
*
* Returns: void
*
* Last Modified: April 11, 2025
*****************************************************************************************/
void setColor(int r, int g, int b) {
    char command[MAX_STRING];
    sprintf(command, "PEN_COLOR %d %d %d\n", r, g, b);
    robot.Send(command);
}

/****************************************************************************************
* Function: setPen
*
* Description:
*	Sets the pen position for the SCARA robot.
*
* Inputs:
*	pen - Pen position ('u' for up, 'd' for down)
*
* Returns: void
*
* Last Modified: April 11, 2025
*****************************************************************************************/
void setPen(char pen) {
    if (pen=='u') {
        robot.Send("PEN_UP\n");
    }else if (pen=='d') {
        robot.Send("PEN_DOWN\n");
    }else {
        printf("Invalid pen: \'%c\'\n",pen);
    }

}

/****************************************************************************************
* Function: setSpeed
*
* Description:
*	Sets the motor speed for the SCARA robot.
*
* Inputs:
*	speed - Motor speed ('H' for high, 'M' for medium, 'L' for low)
*
* Returns: void
*
* Last Modified: April 11, 2025
*****************************************************************************************/
void setSpeed(char speed) {
    if (speed == 'H' || speed == 'h') {
        robot.Send("MOTOR_SPEED HIGH\n");
    } else if (speed == 'M' || speed == 'm') {
        robot.Send("MOTOR_SPEED MEDIUM\n");
    } else if (speed == 'L' || speed == 'l') {
        robot.Send("MOTOR_SPEED LOW\n");
    }else {
        printf("Invalid motor speed: \'%c\'\n",speed);
    }
}


/****************************************************************************************
* Function: scaraSetState
*
* Description:
*	Sets the SCARA robot state. Only sends commands for values that have changed
*   since the last call.
*
* Inputs:
*	scaraState - SCARA robot state
*
* Returns: void
*
* Last Modified: April 22, 2025
*****************************************************************************************/
void scaraSetState(SCARA_ROBOT scaraState) {
    // Static SCARA_ROBOT to hold previous state
    static int initialized = 0;
    static SCARA_ROBOT prevState;
    
    // Initialize prevState with invalid values on first call
    if (!initialized) {
        prevState.armPos.theta1 = 1e9;
        prevState.armPos.theta2 = 1e9;
        prevState.armPos.x = 1e9;
        prevState.armPos.y = 1e9;
        prevState.armPos.armSol = -1;
        prevState.toolPos.penPos = '\0';
        prevState.toolPos.penColor.r = -1;
        prevState.toolPos.penColor.g = -1;
        prevState.toolPos.penColor.b = -1;
        prevState.motorSpeed = '\0';
        initialized = 1;
    }

    // Update joint angles only if changed
    if (scaraState.armPos.theta1 != prevState.armPos.theta1 || 
        scaraState.armPos.theta2 != prevState.armPos.theta2) {
        rotate(scaraState.armPos.theta1, scaraState.armPos.theta2);
        prevState.armPos.theta1 = scaraState.armPos.theta1;
        prevState.armPos.theta2 = scaraState.armPos.theta2;
    }

    // Update pen position only if changed
    if (scaraState.toolPos.penPos != prevState.toolPos.penPos) {
        setPen(scaraState.toolPos.penPos);
        prevState.toolPos.penPos = scaraState.toolPos.penPos;
    }

    // Update pen color only if changed
    if (scaraState.toolPos.penColor.r != prevState.toolPos.penColor.r ||
        scaraState.toolPos.penColor.g != prevState.toolPos.penColor.g ||
        scaraState.toolPos.penColor.b != prevState.toolPos.penColor.b) {
        setColor(scaraState.toolPos.penColor.r, scaraState.toolPos.penColor.g, scaraState.toolPos.penColor.b);
        prevState.toolPos.penColor.r = scaraState.toolPos.penColor.r;
        prevState.toolPos.penColor.g = scaraState.toolPos.penColor.g;
        prevState.toolPos.penColor.b = scaraState.toolPos.penColor.b;
    }

    // Update motor speed only if changed
    if (scaraState.motorSpeed != prevState.motorSpeed) {
        setSpeed(scaraState.motorSpeed);
        prevState.motorSpeed = scaraState.motorSpeed;
    }
}
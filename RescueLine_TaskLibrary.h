/*********************************
Task Library - Li Hong

------------------------
Motor Port Configuration
------------------------
5 : 3 Reduction Ratio -> 5 motor rotation = 3 wheel rotation
Port A - Claw Motor
Port B - Left Motor
Port C - Right Motor

-------------------------
Sensor Port Configuration
-------------------------
Port 1 - Right Light Sensor
Port 2 - Left Light Sensor
Port 3 - SMUX
Channel 2 - Right Colour Sensor
Channel 3 - Left Colour Sensor
Port 4 - Port Splitter
- IMU
- Light Sensor Array
- NXT CAM
*********************************/
/********************
Line Follow Variables
********************/
int Kp = 10;
int Tp = 35;
const int MINIMAL_ERROR = 5;
int error;

int encoderB;
int encoderC;

/***************
Rescue Variables
***************/
#define WALL			1 //integer to denote wall
#define PLATFORM	2	//integer to denote platform

/***************
Sensor Variables
***************/
//I2C Address locations
#define LSA_I2C_ADDRESS 		0x14	//I2C Address for Light Sensor Array
#define LSA_CALIBRATED  		0x42	//I2C Address for calibrated sensor values

#define IMU_I2C_ADDRESS 		0x22	//I2C Address for IMU
#define IMU_TILT_ALL_AXIS		0x42	//I2C Address for Tilt sensor values

//Light Sensor Array
byte LSA_I2CRequest[3] = {2, LSA_I2C_ADDRESS, LSA_CALIBRATED};
byte LSALightValue[8];

int numberOfsensorsOnwhite;
const int MIN_WHITE = 15;
const int MAX_BLACK = 6;

/******************
IMU Tilt

x Tilt - Element 1
y Tilt - Element 2
z Tilt - Element 3
*****************/
byte IMU_Y_TILT_I2CRequest[3] = {2, IMU_I2C_ADDRESS, IMU_TILT_ALL_AXIS};
byte TiltReading[3];
int angle = 0;

const int FLAT_ANGLE = 2;

const int MIN_ANGLE_OF_ELEVATION = 20;
const int MAX_ANGLE_OF_ELEVATION = 28;

const int MIN_ANGLE_OF_DEPRESSION = -12;
const int MAX_ANGLE_OF_DEPRESSION = -26;

/*********************
Light Sensor Variables
*********************/
const int MIN_WHITE_RIGHT = 42;
const int MIN_WHITE_LEFT = 42;

const int MAX_BLACK_RIGHT = 38;
const int MAX_BLACK_LEFT = 38;

const int MIN_SILVER_RIGHT = 58;
const int MIN_SILVER_LEFT = 60;

/************************************
Physical Measurments (in millimetres)
************************************/
const int WHEEL_DIAMETER = 55;
const int DIAMETER_OF_ROTATION = 220;

const int CHECKPOINT_ERROR = 150; //error factor for checkpoint

/*****************
Robot Claw Encoder
*****************/
const int CLAW_LOWER_ENCODER = -270;
const int CLAW_HALF_RAISE_ENCODER = -145;

int taskNo;

/**************************************************
Procedure Libraries

- This program uses open-source code from Xander's
driver suite which can be found here:
https://sourceforge.net/projects/rdpartyrobotcdr/
**************************************************/
#include "common.h"
#include "mindsensors-ev3smux.h"
#include "NXTCamDriver.h"
#include "RescueLine_ProgramLibrary.h"

/***********
Sensor Tasks
***********/
task ReadSensors()
{
	initSensors();
	while(true)
	{
		/**************************
		Port 3 - Colour Sensor Read
		**************************/
		EV3SensorReadings(colourLeft);
		EV3SensorReadings(colourRight);

		/************************
		Port 4 - I2C Sensors Read
		************************/
		I2CSensorReadingsRequest(LSA_I2CRequest, LSALightValue, 8);
		I2CSensorReadingsRequest(IMU_Y_TILT_I2CRequest, TiltReading, 3);

		numBlobs = countBlobs();
		mergeBlobs();

		//Light Sensor Array
		for(int i = 0; i < 8; i++)
		{
			LSAReadings(LightSensorArray[i], LSALightValue[i]);
		}

		//Tilt to Angle
		if(TiltReading[1] != 0)
		{
			angle = radiansToDegrees(asin((126 - abs(TiltReading[1])) / 126.00000)) * TiltReading[1]/abs(TiltReading[1]);
		}

		//Display sensor values - debugging
		/*for(int i = 0; i < 8; i++)
		{
		nxtDisplayCenteredTextLine(i, "%d, %d", i, LightSensorArray[i].lightReading);
		}*/
		//nxtDisplayCenteredTextLine(0, "LL: %d", SensorValue(lightLeft));
		//nxtDisplayCenteredTextLine(1, "LR: %d", SensorValue(lightRight));
		//nxtDisplayCenteredTextLine(2, "MID: %d", colourMiddle.colour);
		//nxtDisplayCenteredTextLine(3, "CR: %d", colourRight.colour);
		//nxtDisplayCenteredTextLine(4, "CL: %d", colourLeft.colour);
		//nxtDisplayCenteredTextLine(5, "Y: %d", TiltReading[1]);
		//nxtDisplayCenteredTextLine(6, "Angle: %d", angle);
	}
}

void MotorSetup()
{
	//Prepare drive motors for line follow
	MotorPIDToggle(false);
	bMotorReflected[motorB] = true;
	bMotorReflected[motorC] = true;

	//Reset motor encoders
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
}

void Obstacle()
{
	stopAllMotors();
	MotorPIDToggle(true);

	//Move away from obstacle
	while(nI2CStatus[portSplitter] == ERR_COMM_BUS_ERR)
	{
		motor[motorB] = -20;
		motor[motorC] = -20;
	}
	RobotDriveStraight(40, -20, false);
	wait1Msec(100);

	//Break from line
	RobotTurn(-60, 25, false);
	wait1Msec(100);

	RobotDriveStraight(30, 20, false);

	clearTimer(T1);
	while(SensorValue(lightLeft) >= MIN_WHITE_LEFT || time1[T1] < 3000) //Navigate around obstacle
	{
		if(nI2CStatus[portSplitter] == ERR_COMM_BUS_ERR) //move away from obstacle if side microswitch is pressed
		{
			motor[motorB] = 40;
			motor[motorC] = -15;
			wait1Msec(100);
		}
		else
		{
			motor[motorB] = -15;
			motor[motorC] = 40;
		}
	}

	//prevent end of robot from clipping
	motor[motorB] = 60;
	motor[motorC] = -20;
	wait1Msec(800);

	motor[motorB] = 20;
	motor[motorC] = 20;
	wait1Msec(400);

	motor[motorB] = -20;
	motor[motorC] = 60;
	wait1Msec(600);
	stopAllMotors();

	//Move back to line
	while(SensorValue(lightRight) >= MIN_WHITE_RIGHT)
	{
		motor[motorB] = -20;
		motor[motorC] = -20;
	}
	RobotDriveStraight(20, 20, false);

	//Regain line
	while(SensorValue(lightLeft) >= MIN_WHITE_LEFT)
	{
		motor[motorB] = 25;
		motor[motorC] = -25;
	}
	motor[motorB] = -25;
	motor[motorC] = 25;
	wait1Msec(250);
	stopAllMotors();

	RobotDriveStraight(30, -20, false);
}

void BrokenLine()
{
	//Lose line
	nMotorEncoder[motorB] = 0;
	numberOfsensorsOnwhite = 0;
	while(numberOfsensorsOnwhite != 8)
	{
		motor[motorB] = Tp;
		motor[motorC] = Tp;

		numberOfsensorsOnwhite = 0;
		for(int i = 0; i < 8; i++)
		{
			if(LightSensorArray[i].OnWhite)
			{
				numberOfsensorsOnwhite += 1;
			}
		}
	}
	wait1Msec(500);

	//Regain line on opposite side or until reaching approximately the end of the tile
	while(numberOfsensorsOnwhite > 6 && nMotorEncoder[motorB] < 700)
	{
		numberOfsensorsOnwhite = 0;
		for(int i = 0; i < 8; i++)
		{
			if(LightSensorArray[i].OnWhite)
			{
				numberOfsensorsOnwhite += 1;
			}
		}
	}
	wait1Msec(300);
	stopAllMotors();

	//search for line by checking sides
	nMotorEncoder[motorB] = 0;
	while(nMotorEncoder[motorB] < 600 && !SensorOnBlack()) //look for line on right side of robot
	{
		motor[motorB] = 25;
		motor[motorC] = -25;
	}

	if(!SensorOnBlack())
	{
		RobotDriveStraight(30, 20, false);
		wait1Msec(100);

		if(!SensorOnBlack())
		{
			RobotDriveStraight(30, -20, false);
			wait1Msec(100);

			nMotorEncoder[motorB] = 0;
			while(nMotorEncoder[motorB] > -1200 && !SensorOnBlack()) //look for line on left side of robot
			{
				motor[motorB] = -25;
				motor[motorC] = 25;
			}
		}
	}
	wait1Msec(200);
	stopAllMotors();

	//Regain line
	while(LightSensorArray[3].OnWhite || LightSensorArray[4].OnWhite)
	{
		motor[motorB] = 20;
		motor[motorC] = 20;
	}
	wait1Msec(400);
	stopAllMotors();

	nMotorEncoder[motorB] = 0;
	while(SensorValue(lightLeft) >= MIN_WHITE_LEFT && SensorValue(lightRight) >= MIN_WHITE_RIGHT && nMotorEncoder[motorB] < 300)
	{
		motor[motorB] = 30;
		motor[motorC] = -30;
	}
	stopAllMotors();

	if(SensorValue(lightLeft) >= MIN_WHITE_LEFT && SensorValue(lightRight) >= MIN_WHITE_RIGHT) //turn other way
	{
		while(SensorValue(lightLeft) >= MIN_WHITE_LEFT && SensorValue(lightRight) >= MIN_WHITE_RIGHT && nMotorEncoder[motorB] > -300)
		{
			motor[motorB] = -30;
			motor[motorC] = 30;
		}
		stopAllMotors();
	}
}

void RegainLine()
{
	MotorPIDToggle(true);

	//Move back to line
	while(LightSensorArray[3].OnWhite || LightSensorArray[4].OnWhite)
	{
		motor[motorB] = -20;
		motor[motorC] = -20;
	}
	stopAllMotors();

	nMotorEncoder[motorB] = 0;
	while(nMotorEncoder[motorB] < 600 && SensorValue(lightRight) >= MIN_WHITE_RIGHT) //Check for line on the right
	{
		motor[motorB] = 25;
		motor[motorC] = -25;
	}
	stopAllMotors();

	if(SensorValue(lightRight) >= MIN_WHITE_RIGHT)
	{
		while(nMotorEncoder[motorB] > -600 && SensorValue(lightLeft) >= MIN_WHITE_LEFT) //Check for line on the left
		{
			motor[motorB] = -25;
			motor[motorC] = 25;
		}
		stopAllMotors();

		if(SensorValue(lightLeft) >= MIN_WHITE_LEFT) //No line on sides so broken line
		{
			while(nMotorEncoder[motorB] < 0) //rotate to centre
			{
				motor[motorB] = 25;
				motor[motorC] = -25;
			}
			stopAllMotors();

			BrokenLine();
		}
	}
}

void InclinationLineFollow()
{
	stopAllMotors();
	MotorPIDToggle(true);

	if(angle >= MIN_ANGLE_OF_ELEVATION && angle <= MAX_ANGLE_OF_ELEVATION) //Travelling Upwards
	{
		RobotClawMotor(CLAW_LOWER_ENCODER, -30); //lower claw for centre of gravity

		while(angle > FLAT_ANGLE) //if-else line follow up quickly until on a roughly flat surface
		{
			if(SensorValue(lightRight) > MIN_WHITE_RIGHT)
			{
				if(SensorValue(lightLeft) > MIN_WHITE_LEFT)
				{
					motor[motorB] = Tp + 10;
					motor[motorC] = Tp + 10;
				}
				else
				{
					motor[motorB] = -20;
					motor[motorC] = 20;
				}
			}
			else
			{
				if(SensorValue(lightLeft) > MIN_WHITE_LEFT)
				{
					motor[motorB] = 20;
					motor[motorC] = -20;
				}
				else
				{
					motor[motorB] = Tp + 10;
					motor[motorC] = Tp + 10;
				}
			}
		}
		stopAllMotors();
		wait1Msec(100);
		RobotClawMotor(0, 20);
		wait1Msec(500);

		RobotDriveStraight(50, -20, false); //allow colour sensors to detect intersections
	}
	else if(angle >= MAX_ANGLE_OF_DEPRESSION && angle <= MIN_ANGLE_OF_DEPRESSION) //Travelling Downwards
	{
		while(angle < FLAT_ANGLE) //travel down slope slowly
		{
			if(SensorValue(lightRight) > MIN_WHITE_RIGHT)
			{
				if(SensorValue(lightLeft) > MIN_WHITE_LEFT)
				{
					motor[motorB] = 20;
					motor[motorC] = 20;
				}
				else
				{
					motor[motorB] = -25;
					motor[motorC] = 20;
				}
			}
			else
			{
				if(SensorValue(lightLeft) > MIN_WHITE_LEFT)
				{
					motor[motorB] = 20;
					motor[motorC] = -25;
				}
				else
				{
					motor[motorB] = 20;
					motor[motorC] = 20;
				}
			}
		}
		stopAllMotors();
		wait1Msec(200);

		RobotDriveStraight(50, -20, false); //allow colour sensors to detect intersections
	}
}

void IntersectionTurns()
{
	stopAllMotors();
	MotorPIDToggle(true);
	RobotDriveStraight(20, -25, false);

	playSound(soundFastUpwardTones);
	wait1Msec(300);

	int turnType = 0;
	if(colourLeft.colour == BLACK || colourRight.colour == BLACK) //Green in front of black line -> forward
	{
		turnType = 4;
	}
	else //move back to check intersection
	{
		RobotDriveStraight(30, 25, false);
		wait1Msec(300);
	}

	if(colourLeft.colour == GREEN && colourRight.colour == GREEN) //U-Turn
	{
		turnType = 1;
	}
	else if(colourLeft.colour == GREEN) //Turn left
	{
		//check for accidental detection while rounding corner
		RobotTurn(15, 30, false);
		wait1Msec(100);

		if(colourRight.colour == GREEN)
		{
			turnType = 1;
		}
		else if(colourLeft.colour != BLACK) //Correct detection
		{
			turnType = 2;
		}
		else
		{
			turnType = 4;
		}
	}
	else if(colourRight.colour == GREEN) //Turn Right
	{
		//check for accidental detection while rounding corner
		RobotTurn(-15, 30, false);
		wait1Msec(100);

		if(colourLeft.colour == GREEN)
		{
			turnType = 1;
		}
		else if(colourRight.colour != BLACK) //Correct detection
		{
			turnType = 3;
		}
		else
		{
			turnType = 4;
		}
	}

	switch(turnType) //Direction to turn
	{
	case 1: //U-turn
		playSound(soundException);
		RobotDriveStraight(70, 25, false);
		wait1Msec(100);

		RobotTurn(180, 30, false);

		while(SensorValue(lightRight) > MIN_WHITE_RIGHT)
		{
			motor[motorB] = -30;
			motor[motorC] = 30;
		}
		motor[motorB] = 30;
		motor[motorC] = -30;
		wait1Msec(250);
		stopAllMotors();
		break;

	case 2: //Turn Left
		playSound(soundBlip);
		RobotDriveStraight(120, 25, false);

		RobotTurn(45, 30, false);
		while(SensorValue(lightRight) > MIN_WHITE_RIGHT)
		{
			motor[motorB] = -30;
			motor[motorC] = 30;
		}
		stopAllMotors();
		break;

	case 3: //Turn Right
		playSound(soundBeepBeep);
		RobotDriveStraight(120, 25, false);

		RobotTurn(-45, 30, false);
		while(SensorValue(lightLeft) > MIN_WHITE_LEFT)
		{
			motor[motorB] = 30;
			motor[motorC] = -30;
		}
		stopAllMotors();
		break;

	case 4: //Drive forward
		RobotDriveStraight(70, 30, false);
		break;
	}
}

void RightAngleTurns()
{
	stopAllMotors();

	playSound(soundDownwardTones);

	MotorPIDToggle(true); //Turn PID On

	RobotDriveStraight(25, -20, false);

	wait1Msec(100);
	if(colourLeft.colour == BLACK && colourRight.colour == BLACK) //both senosrs on black -> Forward
	{
		playSound(soundLowBuzzShort);
		RobotDriveStraight(70, 30, false);
	}
	else if(colourLeft.colour == BLACK) //Turn Left
	{
		playSound(soundBlip);

		RobotDriveStraight(80, 30, false);

		nMotorEncoder[motorB] = 0;
		motor[motorB] = -30;
		motor[motorC] = 30;
		wait1Msec(100);
		while(SensorValue(lightLeft) >= MIN_WHITE_LEFT && nMotorEncoder[motorB] > -600)
		{
			motor[motorB] = -30;
			motor[motorC] = 30;
		}
		stopAllMotors();

		if(SensorValue(lightLeft) >= MIN_WHITE_LEFT) //Turned wrong way so turn back the other way
		{
			while(SensorValue(lightRight) >= MIN_WHITE_RIGHT)
			{
				motor[motorB] = 30;
				motor[motorC] = -30;
			}
			stopAllMotors();
		}
	}
	else if(colourRight.colour == BLACK) //Turn Right
	{
		playSound(soundBeepBeep);

		RobotDriveStraight(80, 30, false);

		nMotorEncoder[motorB] = 0;
		motor[motorB] = 30;
		motor[motorC] = -30;
		wait1Msec(100);
		while(SensorValue(lightRight) >= MIN_WHITE_RIGHT && nMotorEncoder[motorB] < 600)
		{
			motor[motorB] = 30;
			motor[motorC] = -30;
		}
		stopAllMotors();

		if(SensorValue(lightRight) >= MIN_WHITE_RIGHT) //Turned wrong way so turn back the other way
		{
			while(SensorValue(lightLeft) >= MIN_WHITE_LEFT)
			{
				motor[motorB] = -30;
				motor[motorC] = 30;
			}
			stopAllMotors();
		}
	}
	else //Possible Green Detection
	{
		RobotTurn(18, 30, false);

		if(colourRight.colour != GREEN)
		{
			RobotTurn(-36, 30, false);
		}
		RobotDriveStraight(15, -20, false);
	}
}

void LineFollow()
{
	if(time1[T3] > 500)
	{
		if(encoderB == nMotorEncoder[motorB] && encoderC == nMotorEncoder[motorC]) //robot is stuck
		{
			playSound(soundException);
			MotorPIDToggle(true); //Turn PID ON

			//Lower values to prevent oscillation
			Kp = 3;
			Tp = 20;
		}
		else //Robot is fine
		{
			MotorPIDToggle(false); //Turn PID OFF

			Kp = 10;
			Tp = 30;
		}
		encoderB = nMotorEncoder[motorB];
		encoderC = nMotorEncoder[motorC];
		clearTimer(T3);
	}

	//calculate amount of correction required
	error = SensorValue(lightLeft) - SensorValue(lightRight);

	if(abs(error) <= MINIMAL_ERROR) //No correction for small errors
	{
		error = 0;
	}

	//fix error
	motor[motorB] = Tp + (Kp * error);
	motor[motorC] = Tp - (Kp * error);
}

void Rescue()
{
	playSound(soundLowBuzz);
	ResetCheckpointSystem();

	//Entry
	MotorPIDToggle(true);
	RobotDriveStraight(220, 30, false);
	RobotTurn(-90, 30, false);

	AddCheckpoint(0, 0, false, false);

	//Check for wall on right side of robot
	nMotorEncoder[motorB] = 0;
	while(nI2CStatus[portSplitter] != ERR_COMM_BUS_ERR && nMotorEncoder[motorB] < 277)
	{
		motor[motorB] = 30;
		motor[motorC] = 30;
	}
	stopAllMotors();

	bool wall = nI2CStatus[portSplitter] == ERR_COMM_BUS_ERR;
	while(nMotorEncoder[motorB] > 0)
	{
		motor[motorB] = -30;
		motor[motorC] = -30;
	}
	stopAllMotors();

	RobotTurn(90 * wall, 30, true);

	int existingCheckpoint = 0;
	for(int i = 0; i < 16; i++) //Rescue loop
	{
		if(i > 3) //Branching Search
		{
			if(numberOfcheckpoints > 4) //prevent robot from moving to entrance checkpoint for middle entrances
			{
				for(int j = 0; j < numberOfcheckpoints; j++)
				{
					UpdateCheckpoint(j, Checkpoints[j+1].x, Checkpoints[j+1].y, Checkpoints[j+1].platform, Checkpoints[j+1].obstacle);
				}
				RemoveCheckpoint(numberOfcheckpoints);
			}

			//Calculate Distance to travel
			int nextCheckpoint = existingCheckpoint < 3 ? (existingCheckpoint + 1) : 0;
			int x_Distance = Checkpoints[nextCheckpoint].x - Robot_POS[0];
			int y_Distance = Checkpoints[nextCheckpoint].y - Robot_POS[1];
			int distance = sqrt(pow(x_Distance, 2) + pow(y_Distance, 2));

			int branches = distance <= 600 ? 2 : 3;

			BranchedSearch(distance, branches);
		}
		else //Initial Mapping Search
		{
			StraightLineSearch();
		}

		MoveFromWall();

		//Capture any victims still in front of the robot
		RobotDriveStraight(160, -30, false);
		RobotClawMotor(CLAW_LOWER_ENCODER, 30);
		wait1Msec(500);

		while(!stopDriving())
		{
			motor[motorB] = 30;
			motor[motorC] = 30;
		}
		stopAllMotors();

		RobotClawMotor(CLAW_HALF_RAISE_ENCODER, 30);

		if(SensorValue(lightLeft) >= MIN_SILVER_LEFT || SensorValue(lightRight) >= MIN_SILVER_RIGHT) //entrance detected
		{
			RobotDriveStraight(210, -30, false);
		}
		else
		{
			RobotDriveStraight(100, -30, false);
		}
		RobotClawMotor(0, 30);

		RobotDriveStraight(80, 30, false);

		//Check for existing checkpoint on current tile
		existingCheckpoint = SearchExistingCheckpoints();

		/*while(nNxtButtonPressed == -1)
		{
		nxtDisplayCenteredTextLine(2, "%d", existingCheckpoint);
		nxtDisplayCenteredTextLine(3, "%d, %d, %d", Checkpoints[existingCheckpoint].x, Checkpoints[existingCheckpoint].y, Checkpoints[existingCheckpoint].platform);
		nxtDisplayCenteredTextLine(5, "%d, %d", Robot_POS[0], Robot_POS[1]);
		}*/

		int Options = 0;
		if(existingCheckpoint != numberOfcheckpoints - 1) //checkpoint exists here
		{
			RemoveCheckpoint(numberOfcheckpoints - 1);

			Robot_POS[0] = Checkpoints[existingCheckpoint].x;
			Robot_POS[1] = Checkpoints[existingCheckpoint].y;
			playSound(soundBeepBeep);

			if(Checkpoints[existingCheckpoint].platform) //platform here
			{
				Options = PLATFORM;
			}
			else //wall here
			{
				Options = WALL;
			}
		}
		else //new checkpoint to be placed
		{
			Options = BumperCheck();
		}

		if(Options == WALL) //wall - make 90 degree turn to the left
		{
			RobotDriveStraight(100, 30, false);

			RobotDriveStraight(70, -30, false);

			RobotTurn(90, 30, true);
		}
		else if(Options == PLATFORM) //platform - release victims
		{
			if(!platformFound) //log position of platform
			{
				platformFound = true;
				UpdateCheckpoint(numberOfcheckpoints-1, Checkpoints[numberOfcheckpoints-1].x, Checkpoints[numberOfcheckpoints-1].y, true,  Checkpoints[numberOfcheckpoints-1].obstacle);
			}
			ReleaseVictims();
		}

		//Reverse into wall to align
		motor[motorB] = -30;
		motor[motorC] = -30;
		wait1Msec(1300);
		stopAllMotors();

		RobotDriveStraight(20, 20, false);
	}
	stopAllTasks();
}

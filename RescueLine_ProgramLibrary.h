void UpdateRobot(int Magnitude, bool Direction, bool Position);

/*************************
Sensor Setup

Includes:
- initSensors
- EV3SensorReadings
- LSAReadings
- I2CSensorReadingsRequest
*************************/
typedef enum
{
	NONE = 0,
	BLACK = 1,
	BLUE = 2,
	GREEN = 3,
	YELLOW = 4,
	RED = 5,
	WHITE = 6,
	BROWN = 7
}EV3Colour;

typedef struct
{
	tMUXSensor mSensor;
	int Channel;
	tEV3SensorTypeMode mode;
	EV3Colour colour;
	int lightIntensity;
}ColourSensors;

ColourSensors colourLeft;
ColourSensors colourRight;
tMSEV3 EV3Sensors[2];

typedef struct
{
	int lightReading;
	bool OnWhite;
	bool OnBlack;
}MSLSA;

MSLSA LightSensorArray[8];

void initSensors()
{
	//Prepare I2C Ports
	nI2CBytesReady[EV3SMUX] = 0;
	nI2CBytesReady[portSplitter] = 0;

	while(nI2CStatus[EV3SMUX] == STAT_COMM_PENDING || nI2CStatus[portSplitter] == STAT_COMM_PENDING) {
		wait1Msec(5);
	}

	initCamera();

	//Setup EV3 Colour Sensors
	colourRight.mSensor = msensor_S3_2;
	colourRight.Channel = 0;
	colourRight.mode = colorMeasureColor;

	colourLeft.mSensor = msensor_S3_3;
	colourLeft.Channel = 1;
	colourLeft.mode = colorMeasureColor;
}

void EV3SensorReadings(ColourSensors &sensor)
{
	//Change Sensor Mode
	initSensor(&EV3Sensors[sensor.Channel], sensor.mSensor, sensor.mode);

	if(!readSensor(&EV3Sensors[sensor.Channel])) //EV3 SMUX I2C error fix
	{
		eraseDisplay();
		nxtDisplayCenteredTextLine(4, "Fixing SMUX");

		//Reset Port
		SensorType[EV3SMUX] = sensorLightInactive;
		wait1Msec(100);
		SensorType[EV3SMUX] = sensorI2CCustom;

		//Clear error state with bytes
		clearI2CError(EV3SMUX, MSEV3_I2C_ADDR_CHAN2);
		clearI2CError(EV3SMUX, MSEV3_I2C_ADDR_CHAN3);

		eraseDisplay();
	}

	if(sensor.mode == colorReflectedLight) //Obtain Light Intensity
	{
		sensor.lightIntensity = EV3Sensors[sensor.Channel].light;
	}
	else if(sensor.mode == colorMeasureColor) //Obtain Colour Value
	{
		sensor.colour = EV3Sensors[sensor.Channel].color;
	}
}

void LSAReadings(MSLSA &sensor, byte value)
{
	sensor.lightReading = value;
	sensor.OnWhite = sensor.lightReading >= MIN_WHITE;
	sensor.OnBlack = sensor.lightReading <= MAX_BLACK;
}

void I2CSensorReadingsRequest(byte *I2CRequest, byte *replyBytes, int replySize)
{
	sendI2CMsg(portSplitter, &I2CRequest[0], replySize); //send message request

	while(nI2CStatus[portSplitter] == STAT_COMM_PENDING){
		wait1Msec(5);
	}
	memset(replyBytes, 0, replySize); //clear message storage location

	if(nI2CStatus[portSplitter] != ERR_COMM_BUS_ERR) //read and save message if no error
	{
		readI2CReply(portSplitter, &replyBytes[0], replySize);
	}
	else //try to fix error
	{
		wait1Msec(50);
		eraseDisplay();
		nxtDisplayCenteredTextLine(4, "Fixing LSA/IMU/CAM");

		//Reset Port
		SensorType[portSplitter] = sensorLightInactive;
		wait1Msec(100);
		SensorType[portSplitter] = sensorI2CCustomFastSkipStates;

		//Clear error state with dummy bytes
		clearI2CError(portSplitter, LSA_I2C_ADDRESS);
		clearI2CError(portSplitter, IMU_I2C_ADDRESS);
		clearI2CError(portSplitter, CAM_I2C_ADDRESS);
		eraseDisplay();
	}
}

/****************
Sensor Detections

Includes:
- SensorOnBlack
****************/
bool SensorOnBlack()
{
	if(SensorValue(lightLeft) <= MAX_BLACK_LEFT || SensorValue(lightRight) <= MAX_BLACK_RIGHT) //Light sensors detect black
	{
		return true;
	}
	else if(colourLeft.colour == BLACK || colourRight.colour == BLACK) //Colour sensors detect black
	{
		return true;
	}
	else
	{
		return false;
	}
}

/***********************
Robot Motor and Movement

Includes:
- MotorPIDToggle
- RobotDriveStraight
- RobotTurn
- RobotClawMotor
***********************/
void MotorPIDToggle(bool On) //True = On, False = Off
{
	nMotorPIDSpeedCtrl[motorB] = On ? mtrSpeedReg : mtrNoReg;
	nMotorPIDSpeedCtrl[motorC] = On ? mtrSpeedReg : mtrNoReg;
}

void RobotDriveStraight(int distance, int speed, bool Update)
{
	//distance in millimetres
	int encoder = (5.00/3.00) * ((360 * distance) / (PI * WHEEL_DIAMETER));

	nMotorEncoder[motorB] = 0;
	if(speed != 0)
	{
		while(abs(nMotorEncoder[motorB]) < encoder) //speed < 0: backwards, speed > 0: forwards
		{
			motor[motorB] = speed;
			motor[motorC] = speed;
		}
		stopAllMotors();
	}

	if(Update) //Log Distance travelled
	{
		UpdateRobot(distance, false, true);
	}
}

void RobotTurn(int angle, int speed, bool Update)
{
	int encoder = (5.00/3.00) * ((abs(angle) * DIAMETER_OF_ROTATION) / WHEEL_DIAMETER);

	nMotorEncoder[motorB] = 0;
	while(abs(nMotorEncoder[motorB]) < encoder)
	{
		if(angle > 0) //counter-clockwise
		{
			motor[motorB] = -abs(speed);
			motor[motorC] = abs(speed);
		}
		else //clockwise
		{
			motor[motorB] = abs(speed);
			motor[motorC] = -abs(speed);
		}
	}
	stopAllMotors();

	if(Update) //Log Angle turned
	{
		UpdateRobot(angle, true, false);
	}
}

void RobotClawMotor(int amount, int speed)
{
	clearTimer(T1);
	if(nMotorEncoder[motorA] > amount) //lower
	{
		while(nMotorEncoder[motorA] > amount && time1[T1] < 3000)
		{
			motor[motorA] = -abs(speed);
		}
	}
	else //raise
	{
		while(nMotorEncoder[motorA] < amount && time1[T1] < 3000)
		{
			motor[motorA] = abs(speed);
		}
	}
	wait1Msec(100);
	motor[motorA] = 0;
}

/**************************
Checkpoint System

Includes:
- ResetCheckpointSystem
- AddCheckpoint
- RemoveCheckpoint
- UpdateCheckpoint
- SearchExistingCheckpoints
- UpdateRobot
**************************/
typedef struct
{
	int x;
	int y;
	bool platform;
	bool obstacle;
} Checkpoint;

Checkpoint Checkpoints[6];
int numberOfcheckpoints = 0;
bool platformFound = false;

float Robot_POS[3] = {0, 0, 0}; //coords (x, y, angle), angle within [0, 360]

void ResetCheckpointSystem()
{
	//Reset All Checkpoints
	for(int i = 0; i < 5; i++)
	{
		Checkpoints[i].x = NULL;
		Checkpoints[i].y = NULL;
		Checkpoints[i].platform = false;
		Checkpoints[i].obstacle = false;
	}

	//Reset Robot Position
	for(int i = 0; i < 3; i++)
	{
		Robot_POS[i] = 0;
	}

	numberOfcheckpoints = 0;
}

void AddCheckpoint(int xValue, int yValue, bool OnPlatform, bool OnObstacle)
{
	Checkpoints[numberOfcheckpoints].x = xValue;
	Checkpoints[numberOfcheckpoints].y = yValue;
	Checkpoints[numberOfcheckpoints].platform = OnPlatform;
	Checkpoints[numberOfcheckpoints].obstacle = OnObstacle;

	numberOfcheckpoints += 1;
}

void RemoveCheckpoint(int checkpointNumber)
{
	//clear values on checkpoint
	Checkpoints[checkpointNumber].x = NULL;
	Checkpoints[checkpointNumber].y = NULL;
	Checkpoints[checkpointNumber].platform = false;
	Checkpoints[checkpointNumber].obstacle = false;

	numberOfcheckpoints -= 1;
}

void UpdateCheckpoint(int checkpointNumber, int xValue, int yValue, bool OnPlatform, bool OnObstacle)
{
	//changes values on checkpoint
	Checkpoints[checkpointNumber].x = xValue;
	Checkpoints[checkpointNumber].y = yValue;
	Checkpoints[checkpointNumber].platform = OnPlatform;
	Checkpoints[checkpointNumber].obstacle = OnObstacle;
}

int SearchExistingCheckpoints()
{
	for(int i = 0; i < numberOfcheckpoints-1; i++) //look through each checkpoint for one which approximates robot position
	{
		if(Robot_POS[0] >= Checkpoints[i].x - CHECKPOINT_ERROR && Robot_POS[0] <= Checkpoints[i].x + CHECKPOINT_ERROR)
		{
			if(Robot_POS[1] >= Checkpoints[i].y - CHECKPOINT_ERROR && Robot_POS[1] <= Checkpoints[i].y + CHECKPOINT_ERROR)
			{
				return i;
			}
		}
	}
	return numberOfcheckpoints-1;
}

void UpdateRobot(int Magnitude, bool Direction, bool Position)
{
	if(Direction) //anticlockwise (angle starts from x-axis)
	{
		Robot_POS[2] += Magnitude;

		while(Robot_POS[2] < 0 || Robot_POS[2] > 360) //fix angles not in domain
		{
			if(Robot_POS[2] < 0)
			{
				Robot_POS[2] += 360;
			}
			else
			{
				Robot_POS[2] -= 360;
			}
		}
	}
	else if(Position) //coordinates
	{
		Robot_POS[0] += cosDegrees(Robot_POS[2]) * Magnitude; //x
		Robot_POS[1] += sinDegrees(Robot_POS[2]) * Magnitude; //y
	}
}

/*******************
Rescue Functions

Includes:
- MoveFromWall
- stopDriving
- StraightLineSearch
- BranchedSearch
- BumperCheck
- ReleaseVictims
*******************/
void MoveFromWall()
{
	//doge left
	motor[motorB] = -5;
	motor[motorC] = 60;
	wait1Msec(1000);

	//forward a bit
	motor[motorB] = 30;
	motor[motorC] = 30;
	wait1Msec(300);

	//doge right
	motor[motorB] = 60;
	motor[motorC] = -5;
	wait1Msec(1000);
	stopAllMotors();
}

bool stopDriving()
{
	if(SensorValue(lightLeft) == 0 || SensorValue(lightRight) == 0) //Micro switch pressed
	{
		return true;
	}
	else if(SensorValue(lightLeft) >= MIN_SILVER_LEFT || SensorValue(lightRight) >= MIN_SILVER_RIGHT) //entrance detected
	{
		return true;
	}
	else
	{
		return false;
	}
}

void StraightLineSearch()
{
	RobotClawMotor(CLAW_LOWER_ENCODER, 30);
	wait1Msec(300);

	RobotTurn(-15, 30, false);

	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	clearTimer(T1);
	while(!stopDriving()) //Drive to opposite side
	{
		if(ballDetected() && time1[T1] > 500) //rescue ball when detected
		{
			playSound(soundBeepBeep);

			clearTimer(T2);
			while(time1[T2] < 1000 && !stopDriving())
			{
				motor[motorB] = 40;
				motor[motorC] = 40;
			}
			stopAllMotors();

			if(!stopDriving()) //rescue and continue driving unless reached opposite side
			{
				RobotClawMotor(0, 40);

				wait1Msec(300);
				motor[motorB] = -20;
				motor[motorC] = -20;
				wait1Msec(1000);
				stopAllMotors();

				RobotClawMotor(CLAW_LOWER_ENCODER, 35);
				wait1Msec(500);
				clearTimer(T1);
			}
		}
		else //drive
		{
			motor[motorB] = 40;
			motor[motorC] = 40;
		}
	}
	stopAllMotors();

	bool OnSilver = (SensorValue(lightLeft) >= MIN_SILVER_LEFT || SensorValue(lightRight) >= MIN_SILVER_RIGHT);

	//log distance travelled (remove 150mm if on silver - mimic wall detected)
	int avgEncoder = (nMotorEncoder[motorB] + nMotorEncoder[motorC]) / 2;
	float distance = ((3.00/5.00) * (avgEncoder * PI * WHEEL_DIAMETER) / 360) - 150 * (OnSilver);

	UpdateRobot(distance, false, true);

	AddCheckpoint(Robot_POS[0], Robot_POS[1], false, false);

	//Retract claw
	RobotClawMotor(CLAW_HALF_RAISE_ENCODER, 30);
	RobotDriveStraight(100, -30, false);
	RobotClawMotor(0, 30);
}

void BranchedSearch(int distanceTotravel, int branches)
{
	for(int i = 0; i < branches ; i++)
	{
		RobotClawMotor(CLAW_LOWER_ENCODER, 30);
		wait1Msec(300);

		RobotTurn(-10, 30, false);

		RobotDriveStraight(distanceTotravel / (branches + 1), 40, true); //Move to next branch

		RobotClawMotor(0, 35);
		wait1Msec(500);

		MoveFromWall();
		RobotDriveStraight(150, -30, false);

		RobotTurn(90, 30, false);

		//Reverse into wall to align
		motor[motorB] = -30;
		motor[motorC] = -30;
		wait1Msec(1300);
		stopAllMotors();

		RobotClawMotor(CLAW_LOWER_ENCODER, 30);

		nMotorEncoder[motorB] = 0;
		while(nMotorEncoder[motorB] < 1100) //Search Branch (each branch is accross 2 tiles)
		{
			if(ballDetected()) //rescue ball when detected
			{
				playSound(soundBeepBeep);
				wait1Msec(1000);
				stopAllMotors();

				RobotClawMotor(0, 40);
				wait1Msec(300);

				RobotClawMotor(CLAW_LOWER_ENCODER, 30);
			}
			else //Drive straight
			{
				motor[motorB] = 30;
				motor[motorC] = 30;
			}
		}

		motor[motorB] = 40;
		motor[motorC] = 40;
		wait1Msec(300);

		RobotClawMotor(0, 40);
		stopAllMotors();

		while(nMotorEncoder[motorB] > 0) //Return to walll
		{
			motor[motorB] = -40;
			motor[motorC] = -40;
		}
		motor[motorB] = -30;
		motor[motorC] = -30;
		wait1Msec(1000);
		stopAllMotors();

		RobotDriveStraight(50, 20, false);

		//Turn robot to be parallel to wall
		RobotTurn(-90, 30, false);
	}

	RobotDriveStraight(100, -20, false);
	RobotClawMotor(CLAW_LOWER_ENCODER, 30);
	wait1Msec(300);

	RobotTurn(-10, 30, false);

	while(!stopDriving()) //Drive to opposite wall
	{
		motor[motorB] = 40;
		motor[motorC] = 40;
	}
	stopAllMotors();

	//Log distance travelled
	RobotDriveStraight(distanceTotravel / (branches + 1), 0, true);

	AddCheckpoint(Robot_POS[0], Robot_POS[1], false, false);

	//Retract claw
	RobotClawMotor(CLAW_HALF_RAISE_ENCODER, 30);
	RobotDriveStraight(100, -30, false);
	RobotClawMotor(0, 30);
}

int BumperCheck()
{
	bool Bumps[2] = {false, false}; //1 = front, 2 = left side

	nMotorEncoder[motorB] = 0;
	while(nI2CStatus[portSplitter] != ERR_COMM_BUS_ERR && nMotorEncoder[motorB] < 380)
	{
		motor[motorB] = 30;
		motor[motorC] = 30;
	}
	stopAllMotors();

	Bumps[0] = nI2CStatus[portSplitter] == ERR_COMM_BUS_ERR; //front bumper pressed

	RobotDriveStraight(60, -30, false);
	RobotTurn(45, 30, false);

	nMotorEncoder[motorB] = 0;
	while(nI2CStatus[portSplitter] != ERR_COMM_BUS_ERR && nMotorEncoder[motorB] < 380)
	{
		motor[motorB] = 30;
		motor[motorC] = 30;
	}
	stopAllMotors();

	Bumps[1] = nI2CStatus[portSplitter] == ERR_COMM_BUS_ERR; //front bumper pressed (left side)

	while(nMotorEncoder[motorB] > 0)
	{
		motor[motorB] = -30;
		motor[motorC] = -30;
	}
	stopAllMotors();
	RobotTurn(-45, 30, false);

	if(!Bumps[0] && !Bumps[1] && !platformFound) //platform
	{
		return PLATFORM;
	}
	else //wall
	{
		return WALL;
	}
}

void ReleaseVictims()
{
	//Reposition
	RobotTurn(45, 30, false);
	RobotDriveStraight(130, 30, false);
	RobotTurn(85, 30, false);
	wait1Msec(50);

	//Release victims still being held
	RobotClawMotor(CLAW_LOWER_ENCODER, 30);
	wait1Msec(800);

	for(int i = 0; i < 3; i++) //Release Victims (3 times)
	{
		motor[motorB] = -60;
		motor[motorC] = -60;
		wait1Msec(800);
		stopAllMotors();

		wait1Msec(800);

		motor[motorB] = 40;
		motor[motorC] = 40;
		wait1Msec(700);
	}
	stopAllMotors();
	RobotClawMotor(0, 30);

	//Move to wall
	RobotTurn(-90, 30, false);

	nMotorEncoder[motorB] = 0;
	while(nI2CStatus[portSplitter] != ERR_COMM_BUS_ERR && nMotorEncoder[motorB] < 764)
	{
		motor[motorB] = 30;
		motor[motorC] = 30;
	}
	stopAllMotors();

	RobotTurn(55, 30, false);

	//Manual update of robots new position
	UpdateRobot(300, false, true);
	UpdateRobot(90, true, false);
	UpdateRobot(300, false, true);
}

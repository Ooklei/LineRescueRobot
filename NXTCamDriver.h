/**************************************************
NXT Cam Driver

- Important information about the sensor can be
	found in the userguide on mindsensors

-	This code is adapted from Xander's driver suite
	which can be found here:
	https://sourceforge.net/projects/rdpartyrobotcdr/
**************************************************/
//Important I2C Addresses
#define CAM_I2C_ADDRESS		0x02	//I2C Address for NXT Cam
#define CAM_CMD_REG				0x41	//I2C Address for sending commands
#define CAM_COUNT_REG			0x42	//I2C Address for counting blobs
#define CAM_COORD_REG			0x44	//I2C Address for upper left x coord of blob 1

/****************
NXT Cam Variables
****************/
byte CAM_I2CRequest[4];
byte CAM_I2CReply[4];
int numBlobs = 0;
int minBallblobArea = 16000;

/*********************
Camera System

Includes:
- initCamera
- clearBlob
- BlobMergingAlgorithm
- countBlobs
- mergeBlobs
- ballDetected
*********************/
typedef struct
{
	short upperx;
	short uppery;
	short lowerx;
	short lowery;
	short area;
}blobProperties;

blobProperties blobs[8];

void initCamera()
{
	//Set NXT cam to line tracking
	CAM_I2CRequest[0] = 3;
	CAM_I2CRequest[1] = CAM_I2C_ADDRESS;
	CAM_I2CRequest[2] = CAM_CMD_REG;
	CAM_I2CRequest[3] = 'L';
	sendI2CMsg(portSplitter, &CAM_I2CRequest[0], 0);
	while(nI2CStatus[portSplitter] == STAT_COMM_PENDING) {
		wait1Msec(5);
	}

	//Start tracking on NXT cam
	CAM_I2CRequest[3] = 'E';
	sendI2CMsg(portSplitter, &CAM_I2CRequest[0], 0);
	while(nI2CStatus[portSplitter] == STAT_COMM_PENDING) {
		wait1Msec(5);
	}
}

void clearBlob(int blobNumber) //remove blob
{
	blobs[blobNumber].upperx = 0;
	blobs[blobNumber].uppery = 0;
	blobs[blobNumber].lowerx = 0;
	blobs[blobNumber].lowery = 0;
	blobs[blobNumber].area = 0;
}

int BlobMergingAlgorithm(int blob1, int blob2, int num)
{
	int midpointBlob1X = (blobs[blob1].upperx + blobs[blob1].lowerx) / 2;
	int midpointBlob2X = (blobs[blob2].upperx + blobs[blob2].lowerx) / 2;
	int Blob1halfWidth = (blobs[blob1].lowerx - blobs[blob1].upperx) / 2;
	int Blob2halfWidth = (blobs[blob2].lowerx - blobs[blob2].upperx) / 2;

	bool xIntersect = (Blob1halfWidth + Blob2halfWidth) > abs(midpointBlob1X - midpointBlob2X);
	bool yIntersect = (blobs[blob1].lowery - blobs[blob2].uppery) < 2; //closest y values of 2 blobs are within 2 units of each other

	if(xIntersect && yIntersect) //update blob 1 as new merged blob
	{
		blobs[blob1].upperx = (blobs[blob1].upperx < blobs[blob2].upperx) ? blobs[blob1].upperx : blobs[blob2].upperx;
		blobs[blob1].uppery = blobs[blob1].uppery;
		blobs[blob1].lowerx = (blobs[blob1].lowerx > blobs[blob2].lowerx) ? blobs[blob1].lowerx : blobs[blob2].lowerx;
		blobs[blob1].lowery = blobs[blob2].lowery;
		blobs[blob1].area = abs(blobs[blob1].upperx - blobs[blob1].lowerx) * abs(blobs[blob1].uppery - blobs[blob1].lowery);

		for(int i = blob2; i < num; i++) //shift all blobs down by 1 position
		{
			blobs[i].upperx = blobs[i + 1].upperx;
			blobs[i].uppery = blobs[i + 1].uppery;
			blobs[i].lowerx = blobs[i + 1].lowerx;
			blobs[i].lowery = blobs[i + 1].lowery;
			blobs[i].area = blobs[i + 1].area;
		}

		if(num >= 0) //remove blob at the end
		{
			clearBlob(num);
		}
		return num--;
	}
	return num;
}

int countBlobs()
{
	//Lock Tracking Buffer
	CAM_I2CRequest[0] = 3;
	CAM_I2CRequest[2] = CAM_CMD_REG;
	CAM_I2CRequest[3] = 'J';
	sendI2CMsg(portSplitter, &CAM_I2CRequest[0], 0);
	while(nI2CStatus[portSplitter] == STAT_COMM_PENDING){
		wait1Msec(5);
	}

	//Send and read message of number of blobs on screen
	CAM_I2CRequest[0] = 2;
	CAM_I2CRequest[2] = CAM_COUNT_REG;

	sendI2CMsg(S4, &CAM_I2CRequest[0], 1); //send message request
	while(nI2CStatus[S4] == STAT_COMM_PENDING){
		wait1Msec(5);
	}
	memset(CAM_I2CReply, 0, 1); //clear message storage location

	//Read reply
	readI2CReply(S4, &CAM_I2CReply[0], 1);
	if(CAM_I2CReply[0] > 8)
	{
		return 8;
	}
	else
	{
		return CAM_I2CReply[0];
	}
}

void mergeBlobs()
{
	for(int i = 0; i < 8; i++)
	{
		//Send and read message of coords of blobs
		CAM_I2CRequest[2] = CAM_COORD_REG + i * 5;

		sendI2CMsg(S4, &CAM_I2CRequest[0], 4); //send message request
		while(nI2CStatus[S4] == STAT_COMM_PENDING){
			wait1Msec(5);
		}
		memset(CAM_I2CReply, 0, 4); //clear message storage location

		//Read reply
		readI2CReply(S4, &CAM_I2CReply[0], 4);

		/*Update coords of blobs to a record
		(axis starts in botom left corner, removed maximum 254 coordinate x-axis)*/
		blobs[i].upperx = CAM_I2CReply[0] >= 0? CAM_I2CReply[0] : 254 + CAM_I2CReply[0];
		blobs[i].uppery = 127 - CAM_I2CReply[1];
		blobs[i].lowerx = CAM_I2CReply[2] >= 0  ? CAM_I2CReply[2] : 254 + CAM_I2CReply[2];
		blobs[i].lowery = 127 - CAM_I2CReply[3];
		blobs[i].area = abs(blobs[i].upperx - blobs[i].lowerx) * abs(blobs[i].uppery - blobs[i].lowery);
	}

	//merge blobs
	numBlobs--;
	for(int i = 0; i < numBlobs; i++)
	{
		for(int j = 0; j < numBlobs; j++) //merge blob in position 1 with blobs in postion 2 until unable to
		{
			numBlobs = BlobMergingAlgorithm(i, i+1, numBlobs);
		}
	}

	/*for(int i = 0; i < 8; i++)
	{
		//nxtDisplayTextLine(i, "(%d,%d),(%d,%d)", blobs[i].upperx, blobs[i].uppery, blobs[i].lowerx, blobs[i].lowery, blobs[i].area);
		//nxtDisplayTextLine(i, "%d", blobs[i].area);
	}*/
}

bool ballDetected()
{
	for(int i = 0; i < 8; i++) //look through each blob for ball
	{
		if(blobs[i].area > minBallblobArea)
		{
			return true;
		}
	}
	return false;
}

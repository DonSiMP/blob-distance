///////////////////////////////////////////////////////////////////////////////////
// This code sample demonstrates the use of DUO SDK in your own applications
// For updates and file downloads go to: http://duo3d.com/
// Copyright 2014-2015 (c) Code Laboratories, Inc.  All rights reserved.
///////////////////////////////////////////////////////////////////////////////////
#ifndef SAMPLE_H
#define SAMPLE_H

// Include some generic header files
#include <SDKDDKVer.h>
#include <windows.h>
#include <stdio.h>
#include <conio.h>

// Include DUO API header file
#include <DUOLib.h>

//#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// Some global variables
static DUOInstance _duo = NULL;
static PDUOFrame _pFrameData = NULL;
static HANDLE _evFrame = CreateEvent(NULL, FALSE, FALSE, NULL);

// One and only duo callback function
// It sets the current frame data and signals that the new frame data is ready
static void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	_pFrameData = pFrameData;
	SetEvent(_evFrame);
}

// Opens, sets current image format and fps and starts capturing
static bool OpenDUOCamera(int width, int height, float fps)
{
	if(_duo != NULL)
	{
		// Stop capture
		StopDUO(_duo);
		// Close DUO
		CloseDUO(_duo);
		_duo = NULL;
	}

	// Find optimal binning parameters for given (width, height)
	// This maximizes sensor imaging area for given resolution
	int binning = DUO_BIN_NONE;
	if(width <= 752/2) 
		binning += DUO_BIN_HORIZONTAL2;
	else if(width <= 752/4) 
		binning += DUO_BIN_HORIZONTAL4;
	if(height <= 480/4) 
		binning += DUO_BIN_VERTICAL4;
	else if(height <= 480/2) 
		binning += DUO_BIN_VERTICAL2;

	// Check if we support given resolution (width, height, binning, fps)
	DUOResolutionInfo ri;
	if(!EnumerateResolutions(&ri, 1, width, height, binning, fps))
		return false;

	if(!OpenDUO(&_duo))
		return false;

	char tmp[260];
	// Get and print some DUO parameter values
	GetDUODeviceName(_duo,tmp);
	printf("DUO Device Name:      '%s'\n", tmp);
	GetDUOSerialNumber(_duo, tmp);
	printf("DUO Serial Number:    %s\n", tmp);
	GetDUOFirmwareVersion(_duo, tmp);
	printf("DUO Firmware Version: v%s\n", tmp);
	GetDUOFirmwareBuild(_duo, tmp);
	printf("DUO Firmware Build:   %s\n", tmp);

	// Set selected resolution
	SetDUOResolutionInfo(_duo, ri);

	// Start capture
	if(!StartDUO(_duo, DUOCallback, NULL))
		return false;
	return true;
}

// Waits until the new DUO frame is ready and returns it
static PDUOFrame GetDUOFrame()
{
	if(_duo == NULL) 
		return NULL;
	if(WaitForSingleObject(_evFrame, 1000) == WAIT_OBJECT_0)
		return _pFrameData;
	else
		return NULL;
}

// Stops capture and closes the camera
static void CloseDUOCamera()
{
	if(_duo == NULL)
		return;
	// Stop capture
	StopDUO(_duo);
	// Close DUO
	CloseDUO(_duo);
	_duo = NULL;
}

static void SetExposure(float value)
{
	if(_duo == NULL)
		return;
	SetDUOExposure(_duo, value);
}

static void SetGain(float value)
{
	if(_duo == NULL)
		return;
	SetDUOGain(_duo, value);
}

static void SetLed(float value)
{
	if(_duo == NULL)
		return;
	SetDUOLedPWM(_duo, value);
}

static void SetVFlip(bool value)
{
	if(_duo == NULL)
		return;
	SetDUOVFlip(_duo, value);
}

static void SetCameraSwap(bool value)
{
	if(_duo == NULL)
		return;
	SetDUOCameraSwap(_duo, value);
}

static void SetUndistort(bool value)
{
	if(_duo == NULL)
		return;
	SetDUOUndistort(_duo, value);
}

#pragma pack(push, 1)
typedef struct
{
	unsigned short w, h;
	double left[12];
	double right[12];
}INTRINSICS;

typedef struct
{
	double rotation[9];
	double translation[3];
}EXTRINSICS;
#pragma pack(pop)

static bool GetCameraParameters(INTRINSICS *intr, EXTRINSICS *extr)
{
	if(_duo == NULL)
		return false;
	return (GetDUOIntrinsics(_duo, intr) && GetDUOExtrinsics(_duo, extr));
}

#endif // SAMPLE_H

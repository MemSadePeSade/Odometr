#pragma once
#include <string>
#include <iostream>
#include <boost/filesystem/operations.hpp>

#include <stdio.h>
#include <stddef.h>
#include "opencv2/opencv.hpp"
//#include "ueye.h"

class ReaderFromBoost {
public:	
	ReaderFromBoost(std::string path):p(path),itr(p){}
	bool Read(cv::Mat& img) {
		img = cv::imread(itr->path().string());
		++itr;
		return (itr != end_itr) && !img.empty();
	}
private:	
	boost::filesystem::path p;
	boost::filesystem::directory_iterator itr;
	boost::filesystem::directory_iterator end_itr;
 
};

class ReaderFromPrintf {
public:
	ReaderFromPrintf(const char* img_path):m_img_path(img_path){}
	bool Read(cv::Mat& img) {
		sprintf(m_filename, m_img_path, m_counter);
		img = cv::imread(m_filename);
		++m_counter;
		return !img.empty();
	}
private:	 
	char m_filename[200];
	const char* m_img_path;
	int  m_counter = 0;
};
/*
class ReaderFromEyeWebCam {
public:	
	ReaderFromEyeWebCam() {
		// Open cam and see if it was succesfull
		INT nRet = is_InitCamera(&hCam_internal, NULL);
		if (nRet == IS_SUCCESS) 
			std::cout << "Camera initialized!" << std::endl;
		// Setting the pixel clock to retrieve data
		UINT nPixelClockDefault = 21;
		nRet = is_PixelClock(hCam_internal, IS_PIXELCLOCK_CMD_SET, (void*)&nPixelClockDefault, sizeof(nPixelClockDefault));
		if (nRet == IS_SUCCESS) 
			std::cout << "Camera pixel clock succesfully set!" << std::endl;
		else if (nRet == IS_NOT_SUPPORTED) 
			std::cout << "Camera pixel clock setting is not supported!" << std::endl;

		// Set the color mode of the camera
		INT colorMode = IS_CM_MONO8;
		nRet = is_SetColorMode(hCam_internal, colorMode);
		if (nRet == IS_SUCCESS) 
			std::cout << "Camera color mode succesfully set!" << std::endl;
		// Store image in camera memory --> option to chose data capture method
		// Then access that memory to retrieve the data
		INT displayMode = IS_SET_DM_DIB;
		nRet = is_SetDisplayMode(hCam_internal, displayMode);
	}
	~ReaderFromEyeWebCam() {
		is_ExitCamera(hCam_internal);
	}
	bool Read(cv::Mat& mat, int width, int height) {
		char* pMem = NULL;
		int memID = 0;
		is_AllocImageMem(hCam_internal, width, height, 8, &pMem, &memID);
		// Activate the image memory for storing the frame captured
		// Grabbing the image
		// Getting the data of the frame and push it in a Mat element
		is_SetImageMem(hCam_internal, pMem, memID);
		is_FreezeVideo(hCam_internal, IS_WAIT);
		VOID* pMem_b;
		int retInt = is_GetImageMem(hCam_internal, &pMem_b);
		if (retInt != IS_SUCCESS) 
			std::cout << "Image data could not be read from memory!" << std::endl;
		memcpy(mat.ptr(), pMem_b, mat.cols * mat.rows);
		return true;
	}
private:
	HIDS hCam_internal = 0;
};
*/
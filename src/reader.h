#pragma once
#include <string>
#include <iostream>
#include <boost/filesystem/operations.hpp>

#include <stdio.h>
#include <stddef.h>
#include "opencv2/opencv.hpp"

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

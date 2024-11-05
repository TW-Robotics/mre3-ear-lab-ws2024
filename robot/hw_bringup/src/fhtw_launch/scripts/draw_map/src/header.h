/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    header
date:            25.05.19
version:         1.0
*/

#ifndef HEADER_H_INCLUDED
#define HEADER_H_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>
#include <chrono>
#include <ctime>
#include <iomanip>

void write_header( cv::Mat& map );

#endif

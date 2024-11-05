/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    read_csv
date:            17.05.19
version:         1.1
*/

#ifndef READ_CSV_H_INCLUDED
#define READ_CSV_H_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iostream>

#include "gauss.h"
#include "sign.h"
#include "COLORS.h"

namespace fs = boost::filesystem;

void read_csv_person(std::vector<cv::Point>& pos_person, fs::path p_person, int map_rows, double resulution, cv::Point2f origin);
void read_csv_sign(std::vector<Sign>& pos_sign, fs::path p_sign, int map_rows, double resulution, cv::Point2f origin);
void read_csv_gauss(std::vector<Gauss>& pos_gauss, fs::path p_gauss, int map_rows, double resulution, cv::Point2f origin);

#endif

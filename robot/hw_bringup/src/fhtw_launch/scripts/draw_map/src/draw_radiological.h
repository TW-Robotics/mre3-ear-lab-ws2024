/*
author:          Florian Voglsinger
program name:    draw_map_radiological
date:            18.05.19
version:         1.0
*/

#ifndef DRAW_RADIOLOGICAL_H_INCLUDED
#define DRAW_RADIOLOGICAL_H_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "gauss.h"

cv::Scalar color_spot(double cn);

void draw_radiolocial( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution, int rec_size, bool uSv );

void draw_legend( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution );

void draw_legend_uSv( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution );

#endif

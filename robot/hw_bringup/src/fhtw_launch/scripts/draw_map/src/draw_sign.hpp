/*
 author:          Simon Emsenhuber, Florian Voglsinger
 program name:    map
 date:            03.04.19
 version:         1.0
 g++ main.cpp -o SIFT `pkg-config --cflags --libs opencv` -std=c++11
*/


#ifndef DRAW_SIGN_HPP_INCLUDED
#define DRAW_SIGN_HPP_INCLUDED

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

//function to get the given simbol and draw it into a given picture
//src...image to draw into
//sign_src...given simbol
//position...positon the symbol will be drawn at (middel point of symbol)
//map_resulution...resulution of the given image (m/pix)
//symbol_hight/symbol_width...real size of the object(can be addapted to better fit into the map)(m)
//color_range_lower...lower end of the hsv color range to detect the symbol
//color_range_upper...upper end of the hsv color range to detect the symbol
//color_infill...color of the symbol drawn into the image
//color_boarder...color of the symbol boarder drawn into the image
//boarder_size...width of the drawn boarder(pix)
void draw_sign( cv::Mat& src, cv::Mat sign_src, cv::Point position, double map_resulution,
                double symbol_hight, double symbol_width, cv::Scalar color_range_lower,
                cv::Scalar color_range_upper, cv::Scalar color_infill, cv::Scalar color_boarder,
                int boarder_size);


cv::Scalar color_person();

cv::Scalar color_sign(unsigned int type);

cv::Scalar color_sign_boarder();

void draw_legend(   cv::Mat& src, cv::Mat person, cv::Mat sign, double map_resulution,
                    cv::Scalar color_range_lower_person, cv::Scalar color_range_upper_person,
                    cv::Scalar color_range_lower_sign, cv::Scalar color_range_upper_sign);

#endif

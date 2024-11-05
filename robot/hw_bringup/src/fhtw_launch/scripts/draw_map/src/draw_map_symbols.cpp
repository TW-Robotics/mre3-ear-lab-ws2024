/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    map
date:            03.04.19
version:         1.0
*/

//includes
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <libgen.h>
//#include <boost/filesystem.hpp>
//#include <boost/filesystem/fstream.hpp>
#include "draw_sign.hpp"
#include "read_csv.h"
#include "COLORS.h"
#include "sign.h"
#include "header.h"

#define DEBUG 0
#define DEBUG_IMAGE 0

//namespace fs = boost::filesystem;

/** @function main */
int main( int argc, char** argv )
{
    if( argc != 6 )
    {
        std::cout << FORERED << "Not enough arguments (map, map_config.yaml, pos_person, pos_sign, draw_header(0 = false,1 = true) )" << std::endl;
        return -1;
    }

    int draw_header = 0;
    {
        std::stringstream ss;
        ss << argv[5];
        ss >> draw_header;

        if( draw_header != 0)
        {
            if(draw_header != 1)
            {
                std::cout << FORERED << "draw_header has to be either 0 or 1. draw_header = " << draw_header << std::endl;
                return -1;
            }
        }
    }

    //get path of the executible
    //!!! works only for linux systems
    char result[ PATH_MAX ];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    const char *p;
    if (count != -1)
    {
        p = dirname(result);
    }
  
    std::string path = p;

    //read yaml map file
    YAML::Node config_map = YAML::LoadFile( argv[2] );
    std::string map_filename = config_map["image"].as<std::string>();
    //check if given map and yaml config belong to each other
    std::string given_map_path = argv[1];
    if( given_map_path.find( map_filename ) == std::string::npos )
    {
        std::cout << FORERED << given_map_path << " does not include " << map_filename << " -> map and config do not belong together!!" << std::endl;
        return -1;
    }

    //read map
    cv::Mat map;
    map = cv::imread( given_map_path, 1 );    
    if(! map.data )                              // Check for invalid input
    {
        std::cout << FORERED <<  "Could not open or find the image1" << std::endl ;
        return -1;
    }

    double resulution = config_map["resolution"].as<double>();
    std::vector<double> origin = config_map["origin"].as< std::vector<double> >();

    #if DEBUG_IMAGE
    {
        int origin_x = -origin[0] / resulution;
        int origin_y = map.rows + origin[1] / resulution; 

        //display origin
        std::cout << origin_x << " " << origin_y << std::endl;
        //paint red blob at origin
        cv::Point ori(origin_x - 2, origin_y + 2);
        cv::Point ori2(origin_x + 2, origin_y - 2);
        cv::rectangle(map, ori, ori2, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
    }
    #endif

    cv::Mat human;
    cv::Mat sign;

    //read csv person position
    std::vector<cv::Point> pos_person;
    std::vector<Sign> pos_sign;

    read_csv_person(pos_person, argv[3], map.rows, resulution, cv::Point2f(origin[0], origin[1]) );
    read_csv_sign(pos_sign, argv[4], map.rows, resulution, cv::Point2f(origin[0], origin[1]) );

    #if DEBUG_IMAGE
    //check file imput
    {
        std::vector<cv::Point>::iterator it;
        std::cout << "Person csv file contains" << std::endl;
        for(it = pos_person.begin(); it != pos_person.end(); it++)
        {
            std::cout << *it << std::endl;
        }
    }
    #endif

    #if DEBUG_IMAGE
    //check file imput
    {
        std::vector<Sign>::iterator it;
        std::cout << "Sign csv file contains" << std::endl;
        for(it = pos_sign.begin(); it != pos_sign.end(); it++)
        {
            std::cout << *it << std::endl;
        }
    }
    #endif

    //load the symbols 
    human = cv::imread( path + "/resources/stick.jpg", -1);
    if(! human.data )                              // Check for invalid input
    {
        std::cout <<  FORERED << "Could not open or find the stick figure image" << std::endl ;
        return -1;
    }

    //set the color detection range for the stick figure
    cv::Scalar color_range_lower_person( 0, 0, 0 );
    cv::Scalar color_range_upper_person( 180, 255, 90 );

    sign = cv::imread( path +"/resources/simbol.jpeg", -1);
    if(! sign.data )                              // Check for invalid input
    {
        std::cout << FORERED << "Could not open or find the sign image" << std::endl ;
        return -1;
    }

    //set the color detection range for the sign
    cv::Scalar color_range_lower_sign( 5, 50, 50 );
    cv::Scalar color_range_upper_sign( 15, 255, 255 );

    #if DEBUG_IMAGE
    cv::namedWindow("Source_map", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source_map", map);

    cv::namedWindow("Source_human", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source_human", human);

    cv::namedWindow("Source_kemler_zahl", CV_WINDOW_AUTOSIZE);
    cv::imshow("Source_kemler_zahl", sign);
    #endif

    //draw header
    if(draw_header == 1)
    {
        write_header( map );
    }

    //draw the person
    {
        std::vector<cv::Point>::iterator it;
        int boarder_size_person = 1;
		double symbol_real_hight = 1.8;
		double symbol_real_width = 1.0;
    
        for(it = pos_person.begin(); it != pos_person.end(); it++)
        {
            draw_sign(
                    map,
                    human,
                    *it,
                    resulution,
                    symbol_real_hight,
                    symbol_real_width,
                    color_range_lower_person,
                    color_range_upper_person,
                    color_person(),
                    color_person(),
                    boarder_size_person );
        }
    }

    //draw the signs
    {
        std::vector<Sign>::iterator it_sign;
        int boarder_size_sign = 1;
		double symbol_real_hight = 0.5;
		double symbol_real_width = 0.0;

        for(it_sign = pos_sign.begin(); it_sign != pos_sign.end(); it_sign++)
        {
            if(it_sign->type_ < 0 || it_sign->type_ > 8)
            {
                std::cout << FORERED <<  "Type of sign unknown: " << it_sign->type_ << std::endl ;
                return -1;
            }
            cv::Point buffer(it_sign->x_, it_sign->y_);
            draw_sign(
                    map,
                    sign,
                    buffer,
                    resulution,
                    symbol_real_hight,
                    symbol_real_width,
                    color_range_lower_sign,
                    color_range_upper_sign,
                    color_sign(it_sign->type_),
                    color_sign_boarder(),
                    boarder_size_sign );
        }
    }

	//create legende
    draw_legend(
            map,
            human,
            sign,
            resulution,
            color_range_lower_person,
            color_range_upper_person,
            color_range_lower_sign,
            color_range_upper_sign);

    //cv::namedWindow("Result", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Result", map);

	cv::imwrite( path + "/map_sign.ppm", map );

    cv::waitKey(0);
    return(0);
}



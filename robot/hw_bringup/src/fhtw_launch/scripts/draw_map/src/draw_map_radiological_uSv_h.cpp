/*
author:          Florian Voglsinger
program name:    draw_map_radiological
date:            18.05.19
version:         1.0
*/

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>
#include <libgen.h>

#include "gauss.h"
#include "draw_radiological.h"
#include "read_csv.h"
#include "COLORS.h"
#include "header.h"

#define DEBUG_IMAGE 0

int main( int argc, char** argv )
{
    if(argc != 6)
    {
        std::cout << FORERED << "Not enough arguments (map, ymal, gauss_out, gauss_resulution, draw_header(0 = false,1 = true) )" << std::endl;
        return -1;
    }

    double gauss_resulution = 0.0;
    {
        std::stringstream buf;
        buf << argv[4];
        buf >> gauss_resulution;
        if(gauss_resulution == 0.0)
        {
            std::cout << FORERED << "Gauss resulution has to be a float!!" << std::endl;
            return -1;
        }
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

    #if DEBUG_IMAGE
    cv::namedWindow("src", CV_WINDOW_AUTOSIZE);
    //cv::imshow("src", map);
    #endif

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

    //draw header
    if(draw_header == 1)
    {
        write_header( map );
    }

    //read gauss csv
    std::vector<Gauss> pos_gauss;
    read_csv_gauss(pos_gauss, argv[3], map.rows, resulution, cv::Point2f(origin[0], origin[1]) );

    draw_radiolocial( map, pos_gauss, resulution, (gauss_resulution/resulution), true );

    //cv::namedWindow("Result", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Result", map);

	cv::imwrite( path + "/map_gauss_uSv.ppm", map );

    cv::waitKey(0);
    return(0);
}

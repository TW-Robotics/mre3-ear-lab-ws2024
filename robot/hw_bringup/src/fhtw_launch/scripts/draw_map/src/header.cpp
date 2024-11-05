/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    header
date:            25.05.19
version:         1.0
*/ 

#include "header.h"

void write_header( cv::Mat& map )
{
    std::cout.imbue(std::locale("de_DE.utf-8"));
    std::stringstream ss;

    std::time_t now_c = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
    ss << "Team TAUT - Robbie: " << std::put_time(std::localtime( &now_c ), "%F %T");

    //define fonttype and size
    int fontFace = CV_FONT_HERSHEY_DUPLEX;
    double fontScale = 0.65;
    int thickness = 1;
    int baseline = 0;
    cv::Scalar fontColor(0,0,0);

    cv::Size textSize = cv::getTextSize( ss.str(), fontFace, fontScale, thickness, &baseline );

    //draw box around text
    {
        cv::Point left_up( 0, (map.rows - (textSize.height + 2*baseline) ) );
        cv::Point right_down( (2*baseline + textSize.width), (map.rows) );

        cv::rectangle(map, left_up, right_down, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
        cv::rectangle(map, left_up, right_down, cv::Scalar(0, 0, 0), 1, 8, 0);
    }

    //put text
    cv::putText(map, ss.str(), cv::Point( baseline, (map.rows - baseline) ), fontFace, fontScale, fontColor, thickness, 8, false);
}

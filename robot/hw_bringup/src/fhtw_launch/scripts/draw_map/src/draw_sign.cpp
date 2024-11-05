/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    map
date:            03.04.19
version:         1.0
g++ main.cpp -o SIFT `pkg-config --cflags --libs opencv` -std=c++11
*/

//includes
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include "draw_sign.hpp"

#define DEBUG 0
#define DEBUG_IMAGE 0

/** @function main */
void draw_sign( cv::Mat& src, cv::Mat sign_src, cv::Point position, double map_resulution,
                double symbol_hight, double symbol_width, cv::Scalar color_range_lower,
                cv::Scalar color_range_upper, cv::Scalar color_infill, cv::Scalar color_boarder,
                int boarder_size)
{
    //copy the given symbol to ensure the origibal is not changed
    cv::Mat sign = sign_src.clone();
    std::vector< std::vector<cv::Point> > contours;
    std::vector< std::vector<cv::Point> > contours_outline;
    int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
    //kernel size for canny
    int kernel = 3;
    //calculate the hight/width of the symbel fitting to the image
    double symbol_pix_hight = symbol_hight / map_resulution;
	double symbol_pix_width = symbol_width / map_resulution;
    double scaling_factor = 0.0;

    //create an empty image with the size of the sign
    cv::Mat drawing = cv::Mat::zeros(sign.size(), CV_8UC3);

    //convert the sign into the hsv colorsceem
    cv::cvtColor(sign, sign, cv::COLOR_BGR2HSV);

    //detect the devined color range in the sign image
    cv::inRange(sign, color_range_lower, color_range_upper, sign);

    #if DEBUG_IMAGE
    cv::namedWindow("Detect_Figure", CV_WINDOW_AUTOSIZE);
    cv::imshow("Detect_Figure", sign);
    #endif

    //detect the contours in the given color range
    cv::findContours(sign, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    //detect the edges of the contour in the given color range
    cv::Canny(sign, sign, 0, 30, kernel);
    //detect the contour of the edges -> boarder of the simbol
    cv::findContours(sign, contours_outline, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    #if DEBUG_IMAGE
    for( int i = 0; i < contours_outline.size(); i++)
    {
        cv::drawContours(sign, contours_outline, i, cv::Scalar(255, 0, 0), 1);
    }


    cv::namedWindow("Canny", CV_WINDOW_AUTOSIZE);
    cv::imshow("Canny", sign);
    #endif

    #if DEBUG
    for( int i = 0; i < contours.size(); i++ )
    {
        std::cout << contours[i] << " ";
    } 
    std::cout << std::endl;
    #endif

    //detection of the outlying points
    for(unsigned int i = 0; i < contours.size(); i++ )
    {
        for(unsigned int j = 0; j < contours[i].size(); j++ )
        {
           if(max_x < contours[i][j].x)
           {
              max_x = contours[i][j].x;
           }else if(min_x > contours[i][j].x)
           {
              min_x = contours[i][j].x;
           }else if(max_y < contours[i][j].y)
           {
              max_y = contours[i][j].y;
           }else if(min_y > contours[i][j].y)
           {
              min_y = contours[i][j].y;
           }
        }
    }
    //calculating the size of the symbol
    int hor = max_x-min_x, ver=max_y-min_y;

    //calculating the size of the symbol in relation to the image
    if( symbol_hight != 0.0 )
    {
        scaling_factor = ver / symbol_pix_hight;
    }else if( symbol_width != 0.0 )
    {
        scaling_factor = hor / symbol_pix_width;
    }

    #if DEBUG
    std::cout << "Scaling factor: " << scaling_factor << std::endl;
    #endif

    //scale the simbol to the correct size and move it to the given position within the image
    for(unsigned int i = 0; i < contours.size(); i++ )
    {
        for(unsigned int j = 0; j < contours[i].size(); j++ )
        {
            contours[i][j].x = (int)(contours[i][j].x / scaling_factor + position.x - (hor / ( 2 * scaling_factor ) ) );
            contours[i][j].y = (int)(contours[i][j].y / scaling_factor + position.y - (ver / ( 2 * scaling_factor ) ) );
        }
        #if DEBUG
        std::cout << std::endl << std::endl << "Moved and Scaled: " << std::endl;
        std::cout << contours[i] << std::endl;
        #endif
    } 

    //scale the simbol_boarder to the correct size and move it to the given position within the image
    for(unsigned int i = 0; i < contours_outline.size(); i++ )
    {
        for(unsigned int j = 0; j < contours_outline[i].size(); j++ )
        {
            contours_outline[i][j].x = (int)(contours_outline[i][j].x / scaling_factor + position.x - (hor / (2* scaling_factor) ) );
            contours_outline[i][j].y = (int)(contours_outline[i][j].y / scaling_factor + position.y - (ver / (2* scaling_factor ) ) );
        }
        #if DEBUG
        std::cout << std::endl << std::endl << "Moved and Scaled: " << std::endl;
        std::cout << contours_outline[i] << std::endl;
        #endif
    }

    //draw the symbol into the image
    for(unsigned int i = 0; i < contours.size(); i++ )
    {
        cv::drawContours( src, contours, i, color_infill, -1 );
    }

    //draw the symbol boarder into the image
    for(unsigned int i = 0; i < contours_outline.size(); i++ )
    {
        cv::drawContours( src, contours_outline, i, color_boarder, boarder_size );
    }

    #if DEBUG_IMAGE
    cv::namedWindow("Contour", CV_WINDOW_AUTOSIZE);
    cv::imshow("Contour", src);
    #endif

}

cv::Scalar color_person()
{
    return cv::Scalar(255,0,0);
}

cv::Scalar color_sign_boarder()
{
    return cv::Scalar(0,0,0);
}

cv::Scalar color_sign(unsigned int type)
{
    switch (type)
    {
        case 1:
            return cv::Scalar(0,140,255);
        case 2:
            return cv::Scalar(0,0,255);
        case 3:
            return cv::Scalar(0,255,0);
        case 4:
            return cv::Scalar(255,0,0);
        case 5:
            return cv::Scalar(255,255,0);
        case 6:
            return cv::Scalar(0,255,255);
        case 7:
            return cv::Scalar(255,0,255);
        case 8:
            return cv::Scalar(140,140,140);
        case 9:
            return cv::Scalar(40,89,50);
        default:
            return cv::Scalar(0,0,0);
    }
}

void draw_legend(cv::Mat& map, cv::Mat person, cv::Mat sign, double map_resulution,
    cv::Scalar color_range_lower_person, cv::Scalar color_range_upper_person,
    cv::Scalar color_range_lower_sign, cv::Scalar color_range_upper_sign)
{
    std::vector<std::string> legend_text;
    legend_text.push_back("   Person");
    legend_text.push_back("   Tafel unbekannt");
    legend_text.push_back("   70 RN Stoff");
    legend_text.push_back("   72 RN Gas");
    legend_text.push_back("   723 RN Gase, brennbar");
    legend_text.push_back("   73 RN Stoff entzuendbar");
    legend_text.push_back("   74 RN fester Stoff entzuendbar");
    legend_text.push_back("   75 RN Stoff brandfoerdernd");
    legend_text.push_back("   76 RN Stoff giftig");
    legend_text.push_back("   78 RN Stoff aetzend");
    std::stringstream s_buf;
    //s_buf << "Scale: 1:" << int(1 / map_resulution*100);
    s_buf << "Scale:";

    //define the fonttype and size
    int fontFace = CV_FONT_HERSHEY_DUPLEX;
    double fontScale = 0.65;
    int thickness = 1;
    int baseline=0;
    cv::Scalar fontColor(0,0,0);
    int thickness_scale = 2;

    //determine the text size
    std::vector<cv::Size> textSize;
    {
        std::vector<std::string>::iterator it;
        for(it = legend_text.begin(); it != legend_text.end(); it++)
        {
            textSize.push_back( cv::getTextSize(*it, fontFace, fontScale, thickness, &baseline) );
        }
    }
    cv::Size scale_size = cv::getTextSize(s_buf.str(), fontFace, fontScale, thickness_scale, &baseline);

    //dertermine the text width
    int text_width = 0;
    {
        std::vector<cv::Size>::iterator it;
        for(it = textSize.begin(); it != textSize.end(); it++)
        {
            if(it->width > text_width)
            {
                text_width = it->width;
            }
        }

        if(text_width < scale_size.width)
        {
            text_width = scale_size.width;
        }
    }

    baseline += thickness;

    //create bigger image for the legend
    {
        cv::Mat map_large( map.rows, ( map.cols + text_width + baseline + textSize[0].height ), map.type() );
        //cv::Vec3b color_unknown = map.at<cv::Vec3b>( cv::Point(0, 0) );
        cv::Vec3b color_unknown( 205, 205, 205 );
        map_large.setTo( color_unknown );
        map.copyTo( map_large( cv::Rect( 0, 0, map.cols, map.rows ) ) );
        cv::swap(map, map_large);
    }

    //calculate the origin points of the text
    std::vector<cv::Point> textOrig_cords;
    {
        std::vector<cv::Size>::iterator it;
        int i = 1;
        for(it = textSize.begin(); it != textSize.end(); it++, i++)
        {
            cv::Point buf( (map.cols - text_width - baseline), (i * it->height + i * baseline) );
            textOrig_cords.push_back( buf );
        } 
    }

    //calcualte scale origin point
    cv::Point scale_ori = textOrig_cords.back();
    scale_ori.x = map.cols - ( ( text_width + baseline + textSize[0].height ) / 2 ) - ( scale_size.width / 2 );
    scale_ori.y += textSize[0].height;
    scale_ori.y += 2*baseline;

    //draw box around text
    {
        cv::Point buf = scale_ori;
        cv::Point left_up( ( map.cols - text_width - baseline - textSize[0].height ), 0);
        cv::Point right_down( ( map.cols - 1 ), buf.y + 2 * baseline + 10); 

        cv::rectangle(map, left_up, right_down, cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
        cv::rectangle(map, left_up, right_down, cv::Scalar(0, 0, 0), 1, 8, 0);
    }

    //draw scale bar
    {
        int resul_to_scale_x2 = 1 / map_resulution * 2;
        std::string text = "0m";
        double scale_font_scale = fontScale / 2;
        cv::Size scale_size = cv::getTextSize(text, fontFace, scale_font_scale, thickness, &baseline);

        cv::Point left_down( ( map.cols - ( (text_width + baseline + textSize[0].height)/2 ) - ( 3*resul_to_scale_x2/2 ) ), ( scale_ori.y + 2*baseline + 10) );
        cv::Point right_up( ( left_down.x + resul_to_scale_x2 ), ( left_down.y + 5 ) );
        cv::rectangle(map, left_down, right_up, cv::Scalar(0, 0, 0), 1, 8, 0);
        cv::Point textOrig( ( left_down.x - scale_size.width/2 ), ( right_up.y - 5 - baseline ) );
        cv::putText(map, text, textOrig, fontFace, scale_font_scale, fontColor, thickness, 8, false);

        text = "2m";
        textOrig.x = ( right_up.x - scale_size.width/2 );
        cv::putText(map, text, textOrig, fontFace, scale_font_scale, fontColor, thickness, 8, false);

        cv::Point left_up( right_up  );
        cv::Point right_down( ( left_up.x + resul_to_scale_x2 ), ( left_up.y - 5 ) );
        cv::rectangle(map, left_up, right_down, cv::Scalar(0, 0, 0), CV_FILLED, 8, 0);
        text = "4m";
        textOrig.x = ( right_down.x - scale_size.width/2 );
        cv::putText(map, text, textOrig, fontFace, scale_font_scale, fontColor, thickness, 8, false);

        left_down.x = right_down.x;
        right_up.x = left_down.x + 40;
        cv::rectangle(map, left_down, right_up, cv::Scalar(0, 0, 0), 1, 8, 0);
        text = "6m";
        textOrig.x = ( right_up.x - scale_size.width/2 );
        cv::putText(map, text, textOrig, fontFace, scale_font_scale, fontColor, thickness, 8, false);
    }

    //write text and draw signs
    {
        std::vector<std::string>::iterator it;
        std::vector<cv::Point>::iterator it_o;
        int i = 1;
        int j = 1;
        for(it = legend_text.begin(), it_o = textOrig_cords.begin(); ( it != legend_text.end() && it_o != textOrig_cords.end() ); it++, it_o++)
        {
            cv::putText(map, *it, *it_o, fontFace, fontScale, fontColor, thickness, 8, false);
            cv::Point buf;
            switch (i)
            {
                case 1:
                    buf.x = it_o->x;
                    buf.y = ( it_o->y - textSize[0].height / 2 );
                    draw_sign(
                            map,
                            person,
                            buf,
                            map_resulution,
                            ( (textSize[0].height + 7) * map_resulution ),
                            0.0,
                            color_range_lower_person,
                            color_range_upper_person,
                            color_person(),
                            color_person(),
                            1);
                    i++;
                    break;
                default:
                    buf.x = it_o->x;
                    buf.y = (it_o->y - textSize[0].height / 2 );
                    draw_sign(
                            map,
                            sign,
                            buf,
                            map_resulution,
                            ( textSize[0].height * map_resulution ),
                            0.0,
                            color_range_lower_sign,
                            color_range_upper_sign,
                            color_sign(j),
                            color_sign_boarder(),
                            1);
                    j++;
            }
        }
        //write scale
        cv::putText(map, s_buf.str(), scale_ori, fontFace, fontScale, fontColor, thickness_scale, 8, false);
    }

}


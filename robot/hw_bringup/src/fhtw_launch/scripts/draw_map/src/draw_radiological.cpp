/*
author:          Florian Voglsinger
program name:    draw_map_radiological
date:            18.05.19
version:         1.0
*/

#include "draw_radiological.h"

cv::Scalar color_spot(double cn)
{
    if( cn < 0.25 )
    {
        return cv::Scalar( 255, (255*cn*4), 0 );
    }else if( cn < 0.5 )
    {
        return cv::Scalar( ( ( 1-(4*(cn-0.25) ) )*255 ), 255, 0 );
    }else if( cn < 0.75 )
    {
        return cv::Scalar( 0, 255, (4*(cn-0.5)*255) );
    }else
    {
        return cv::Scalar( 0, ( (1-( 4*( cn-0.75 ) ) )*255 ), 255);
    }
}

void draw_radiolocial( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution, int rec_size, bool uSv )
{
    double max_counts = 0;
    double min_counts = 999999999999;

    //get the minimal and the maximal normalized count
    {
        std::vector<Gauss>::iterator it;
        for(it = gauss.begin(); it != gauss.end(); it++)
        {
            if(it->cn_ > max_counts)
            {
                max_counts = it->cn_;
            }
            if(it->cn_ < min_counts)
            {
                min_counts = it->cn_;
            }
        } 
    }

    //draw the rectangels in the map
    {
        std::vector<Gauss>::iterator it;
        double alpha = 0.3;
        cv::Vec3b color_empty(249, 249, 249);
        for(it = gauss.begin(); it != gauss.end(); it++)
        {
            cv::Vec3b color_pix = map.at<cv::Vec3b>( cv::Point(it->x_, it->y_) );
            if( (color_pix[0] > color_empty[0]) && (color_pix[1] > color_empty[1]) && (color_pix[2] > color_empty[2]) )
            {
                cv::Point left_top( (it->x_ - rec_size/2), (it->y_ - rec_size/2) );
                cv::Point right_down( (it->x_ + rec_size/2), (it->y_ + rec_size/2) );

                cv::Mat roi = map( cv::Rect( left_top, right_down ) );
                cv::Mat color(roi.size(), CV_8UC3, color_spot(it->cn_) );
                cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
            }
        }
    }

    //draw legend
    if( uSv == true )
    {
        draw_legend_uSv(map, gauss, map_resulution);
    }else
    {
        draw_legend(map, gauss, map_resulution);
    }
}

void draw_legend( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution )
{
    double max_counts = 0;
    double min_counts = 999999999999;

    //get the minimal and the maximal count
    {
        std::vector<Gauss>::iterator it;
        for(it = gauss.begin(); it != gauss.end(); it++)
        {
            if(it->c_ > max_counts)
            {
                max_counts = it->c_;
            }
            if(it->c_ < min_counts)
            {
                min_counts = it->c_;
            }
        }
    }
    
    //setup text
    std::stringstream counts_upper;
    std::stringstream counts_lower;
    std::stringstream scale;

    counts_upper << "  " << max_counts << " [cps]";
    counts_lower << "  " << min_counts << " [cps]";
    //scale << "Scale: 1:" << int(1/map_resulution*100);
    scale << "Scale:";

    //define fonttype and size
    int fontFace = CV_FONT_HERSHEY_DUPLEX;
    double fontScale = 0.65;
    int thickness = 1;
    int thickness_scale = 2;
    int baseline = 0;
    cv::Scalar fontColor(0,0,0);
    
    //determine the text size
    std::vector<cv::Size> textSize;
    {
        textSize.push_back( cv::getTextSize( counts_upper.str(), fontFace, fontScale, thickness, &baseline ) );
        textSize.push_back( cv::getTextSize( counts_lower.str(), fontFace, fontScale, thickness, &baseline ) );
        textSize.push_back( cv::getTextSize( scale.str(), fontFace, fontScale, thickness_scale, &baseline ) );
    }

    int text_width = 0;
    {
        std::vector<cv::Size>::iterator it;
        for( it = textSize.begin(); it != textSize.end(); it++)
        {
            if( it->width > text_width )
            {
                text_width = it->width;
            }
        }
    }

    baseline += thickness;

    //create bigger image for legend
    {
        cv::Mat map_large( map.rows, ( map.cols + text_width + baseline + textSize[0].height ), map.type() );
        //cv::Vec3b color_unknown = map.at<cv::Vec3b>( cv::Point(0,0) );
        cv::Vec3b color_unknown( 205, 205, 205 );
        map_large.setTo(color_unknown);
        map.copyTo( map_large( cv::Rect( 0, 0, map.cols, map.rows ) ) );
        cv::swap(map, map_large);
    }

    //calculate the origin points of the text
    std::vector<cv::Point> textOrig_cords;
    {
        for(int i = 1 ; i < 11; i++)
        { 
            cv::Point buf( (map.cols - text_width - baseline), (i*textSize[0].height + i*baseline) );
            textOrig_cords.push_back(buf);
        }
    }

    //calculate the origin point of the scale
    cv::Point scale_ori = textOrig_cords.back();
    scale_ori.x = map.cols - ( ( text_width + baseline + textSize[0].height ) / 2 ) - ( textSize.back().width / 2 );
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

    //write scale
    cv::putText(map, scale.str(), scale_ori, fontFace, fontScale, fontColor, thickness_scale, 8, false);    

    //write text and draw signs
    {
        double count_col = 1.0;
        cv::putText(map, counts_upper.str(), textOrig_cords[0], fontFace, fontScale, fontColor, thickness, 8, false);
        cv::putText(map, counts_lower.str(), textOrig_cords.back(), fontFace, fontScale, fontColor, thickness, 8, false);
        std::vector<cv::Point>::iterator it_o;
        for(it_o = textOrig_cords.begin(); it_o != textOrig_cords.end(); it_o++, count_col -= 0.1)
        {
            cv::Point left_down( (it_o->x - textSize[0].height/2), (it_o->y + 2*baseline ) );
            cv::Point right_up( (it_o->x + textSize[0].height/2 + baseline), (it_o->y - textSize[0].height - baseline ) );
            cv::rectangle(map, left_down, right_up, color_spot(count_col), CV_FILLED, 8, 0);
        }
    }
}

void draw_legend_uSv( cv::Mat& map, std::vector<Gauss>& gauss, double map_resulution )
{
    double max_counts = 0;
    double min_counts = 999999999999;

    //get the minimal and the maximal count
    {
        std::vector<Gauss>::iterator it;
        for(it = gauss.begin(); it != gauss.end(); it++)
        {
            if(it->c_ > max_counts)
            {
                max_counts = it->c_;
            }
            if(it->c_ < min_counts)
            {
                min_counts = it->c_;
            }
        }
    }
    
    //setup text
    std::stringstream counts_upper;
    std::stringstream counts_lower;
    std::stringstream scale;

    counts_upper << "  " << max_counts << " [uSv/h]";
    counts_lower << "  " << min_counts << " [uSv/h]";
    //scale << "Scale: 1:" << int(1/map_resulution*100);
    scale << "Scale:";

    //define fonttype and size
    int fontFace = CV_FONT_HERSHEY_DUPLEX;
    double fontScale = 0.65;
    int thickness = 1;
    int thickness_scale = 2;
    int baseline = 0;
    cv::Scalar fontColor(0,0,0);
    
    //determine the text size
    std::vector<cv::Size> textSize;
    {
        textSize.push_back( cv::getTextSize( counts_upper.str(), fontFace, fontScale, thickness, &baseline ) );
        textSize.push_back( cv::getTextSize( counts_lower.str(), fontFace, fontScale, thickness, &baseline ) );
        textSize.push_back( cv::getTextSize( scale.str(), fontFace, fontScale, thickness_scale, &baseline ) );
    }

    int text_width = 0;
    {
        std::vector<cv::Size>::iterator it;
        for( it = textSize.begin(); it != textSize.end(); it++)
        {
            if( it->width > text_width )
            {
                text_width = it->width;
            }
        }
    }

    baseline += thickness;

    //create bigger image for legend
    {
        cv::Mat map_large( map.rows, ( map.cols + text_width + baseline + textSize[0].height ), map.type() );
        //cv::Vec3b color_unknown = map.at<cv::Vec3b>( cv::Point(0,0) );
        cv::Vec3b color_unknown( 205, 205, 205 );
        map_large.setTo(color_unknown);
        map.copyTo( map_large( cv::Rect( 0, 0, map.cols, map.rows ) ) );
        cv::swap(map, map_large);
    }

    //calculate the origin points of the text
    std::vector<cv::Point> textOrig_cords;
    {
        for(int i = 1 ; i < 11; i++)
        { 
            cv::Point buf( (map.cols - text_width - baseline), (i*textSize[0].height + i*baseline) );
            textOrig_cords.push_back(buf);
        }
    }

    //calculate the origin point of the scale
    cv::Point scale_ori = textOrig_cords.back();
    scale_ori.x = map.cols - ( ( text_width + baseline + textSize[0].height ) / 2 ) - ( textSize.back().width / 2 );
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

    //write scale
    cv::putText(map, scale.str(), scale_ori, fontFace, fontScale, fontColor, thickness_scale, 8, false);    

    //write text and draw signs
    {
        double count_col = 1.0;
        cv::putText(map, counts_upper.str(), textOrig_cords[0], fontFace, fontScale, fontColor, thickness, 8, false);
        cv::putText(map, counts_lower.str(), textOrig_cords.back(), fontFace, fontScale, fontColor, thickness, 8, false);
        std::vector<cv::Point>::iterator it_o;
        for(it_o = textOrig_cords.begin(); it_o != textOrig_cords.end(); it_o++, count_col -= 0.1)
        {
            cv::Point left_down( (it_o->x - textSize[0].height/2), (it_o->y + 2*baseline ) );
            cv::Point right_up( (it_o->x + textSize[0].height/2 + baseline), (it_o->y - textSize[0].height - baseline ) );
            cv::rectangle(map, left_down, right_up, color_spot(count_col), CV_FILLED, 8, 0);
        }
    }
}


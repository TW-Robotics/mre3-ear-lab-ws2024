/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    read_csv
date:            17.05.19
version:         1.1
*/

#include "read_csv.h"

void read_csv_person(std::vector<cv::Point>& pos_person, fs::path p_person, int map_rows, double resulution, cv::Point2f origin)
{
    try
    {
        if( fs::exists(p_person) )
        {
            if( !fs::is_regular_file(p_person) )
            { 
                std::cout << FORERED <<  "Given file is not a regular file" << std::endl ;
                return;
            }else
            {
                fs::ifstream f(p_person);
                if( !f.is_open() )
                {
                    std::cout << FORERED <<  "Could not open file " << p_person  << std::endl ;
                    return ;
                }else
                {
                    std::string str_buffer;
                    std::stringstream ss;
                    while( std::getline(f, str_buffer) )
                    {
                        int i = 0;
                        cv::Point2f pos_buffer;
                        ss << str_buffer;
                        str_buffer = "";

                        while( std::getline( ss, str_buffer, ',') )
                        {
                            std::stringstream ss_buffer(str_buffer);
                            switch (i)
                            {
                                case 0:
                                    ss_buffer >> pos_buffer.x;
                                    pos_buffer.x -= origin.x;
                                    break;
                                case 1:
                                    ss_buffer >> pos_buffer.y;
                                    pos_buffer.y = ( origin.y - pos_buffer.y ) * (-1);
                                    break;
                            }
                            i++;
                            str_buffer = "";
                        }
                        ss.clear();
                        ss.str( std::string() );
                        
                        pos_person.push_back( cv::Point( ( pos_buffer.x / resulution ), ( map_rows - pos_buffer.y / resulution ) ) );
                    }
                }

            }
        }else
        {
            std::cout << FORERED <<  "Given file " << p_person << "does not exist" << std::endl ;
            return;
        }
    }
    catch (const fs::filesystem_error& ex)
    {
        std::cout << FORERED <<  "Could not open or find the image1" << std::endl ;

    }
}

void read_csv_sign(std::vector<Sign>& pos_sign, fs::path p_sign, int map_rows, double resulution, cv::Point2f origin)
{
    try
    {
        if( fs::exists(p_sign) )
        {
            if( !fs::is_regular_file(p_sign) )
            { 
                std::cout << FORERED <<  "Given file is not a regular file" << std::endl ;
                return;
            }else
            {
                fs::ifstream f(p_sign);
                if( !f.is_open() )
                {
                    std::cout << FORERED <<  "Could not open file " << p_sign  << std::endl ;
                    return ;
                }else
                {
                    std::string str_buffer;
                    std::stringstream ss;
                    while( std::getline(f, str_buffer) )
                    {
                        int i = 0;
                        unsigned int type = 0;
                        cv::Point2f pos_buffer;
                        ss << str_buffer;
                        str_buffer = "";

                        while( std::getline( ss, str_buffer, ',') )
                        {
                            std::stringstream ss_buffer(str_buffer);
                            switch (i)
                            {
                                case 0:
                                    ss_buffer >> type;
                                    break;
                                case 1:
                                    ss_buffer >> pos_buffer.x;
                                    pos_buffer.x -= origin.x;
                                    break;
                                case 2:
                                    ss_buffer >> pos_buffer.y;
                                    pos_buffer.y = ( origin.y - pos_buffer.y ) * (-1);
                                    break;
                            }
                            i++;
                            str_buffer = "";
                        }
                        ss.clear();
                        ss.str( std::string() );
                        
                        pos_sign.push_back( Sign( type ,( pos_buffer.x / resulution ), ( map_rows - pos_buffer.y / resulution ) ) );
                    }
                }

            }
        }else
        {
            std::cout << FORERED <<  "Given file " << p_sign << "does not exist" << std::endl ;
            return;
        }
    }
    catch (const fs::filesystem_error& ex)
    {
        std::cout << FORERED <<  "Could not open or find the image1" << std::endl ;

    }
}

void read_csv_gauss(std::vector<Gauss>& pos_gauss, fs::path p_gauss, int map_rows, double resulution, cv::Point2f origin)
{
    try
    {
        if( fs::exists(p_gauss) )
        {
            if( !fs::is_regular_file(p_gauss) )
            { 
                std::cout << FORERED <<  "Given file is not a regular file" << std::endl ;
                return;
            }else
            {
                fs::ifstream f(p_gauss);
                if( !f.is_open() )
                {
                    std::cout << FORERED <<  "Could not open file " << p_gauss  << std::endl ;
                    return ;
                }else
                {
                    std::string str_buffer;
                    std::stringstream ss;
                    while( std::getline(f, str_buffer) )
                    {
                        int i = 0;
                        cv::Point2f pos_buffer;
                        double d_buffer_c = 0.0, d_buffer_cn = 0.0;
                        ss << str_buffer;
                        str_buffer = "";

                        while( std::getline( ss, str_buffer, ',') )
                        {
                            std::stringstream ss_buffer(str_buffer);
                            switch (i)
                            {
                                case 0:
                                    ss_buffer >> pos_buffer.x;
                                    pos_buffer.x -= origin.x;
                                    break;
                                case 1:
                                    ss_buffer >> pos_buffer.y;
                                    pos_buffer.y = ( origin.y - pos_buffer.y ) * (-1);
                                    break;
                                case 2:
                                    ss_buffer >> d_buffer_c;
                                    break;
                                case 3:
                                    ss_buffer >> d_buffer_cn;
                                    break;
                            }
                            i++;
                            str_buffer = "";
                        }
                        ss.clear();
                        ss.str( std::string() );
                        
                        pos_gauss.push_back( Gauss( ( pos_buffer.x / resulution ), ( map_rows - pos_buffer.y / resulution ), d_buffer_c, d_buffer_cn  ) );
                    }
                }

            }
        }else
        {
            std::cout << FORERED <<  "Given file " << p_gauss << "does not exist" << std::endl ;
            return;
        }
    }
    catch (const fs::filesystem_error& ex)
    {
        std::cout << FORERED <<  "Could not open or find the image1" << std::endl ;

    }
}

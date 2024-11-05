/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    sign
date:            17.05.19
version:         1.0
*/

#ifndef GAUSS_INCLUDED
#define GAUSS_INCLUDED

#include <iostream>

class Gauss
{
    public:
        Gauss(){};
        Gauss(unsigned int x, unsigned int y, double c, double cn)
        {
            x_ = x;
            y_ = y;
            c_ = c;
            cn_ = cn;
        }
        
        ~Gauss(){};

        unsigned int x_;
        unsigned int y_;
        double c_;
        double cn_;
    protected:

    private:
};

#endif
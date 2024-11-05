/*
author:          Simon Emsenhuber, Florian Voglsinger
program name:    sign
date:            17.05.19
version:         1.0
*/

#ifndef SIGN_INCLUDED
#define SIGN_INCLUDED

#include <iostream>

class Sign
{
    public:
        Sign(){};
        Sign(unsigned int type, unsigned int x, unsigned int y)
        {
            type_ = type;
            x_ = x;
            y_ = y;
        }
        
        ~Sign(){};

        unsigned int type_;
        unsigned int x_;
        unsigned int y_;        

    protected:

    private:

};

std::ostream& operator<<(std::ostream& out, const Sign& sign)
{
    out << "[" << sign.type_ << ", " << sign.x_ << ", " << sign.y_ << "]";
    return out;
}
#endif

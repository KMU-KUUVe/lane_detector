#ifndef MYEXCEPTION_H
#define MYEXCEPTION_H

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

/**
 * @brief 이미지에서 잘못된 인덱스 접근을 하였을 때 사용되는 익셉션 클래스
 * @details 익셉션 클래스를 정의한 이유는 잘못된 인덱스 접근을 한 위치를 정확하고 빠르게 알기 위해서이다.
 * 
 */
class my_out_of_range : public std::out_of_range
{
public:
    my_out_of_range(const std::string &arg, const char *file, int line) :
    std::out_of_range(arg) {
        std::ostringstream o;
        o << file << ":" << line << ": my_out_of_range: " << arg;
        msg = o.str();
    }
    ~my_out_of_range() throw() {}
    const char *what() const throw() {
        return msg.c_str();
    }

private:
    std::string msg;
};
#define throw_my_out_of_range(arg) throw my_out_of_range(arg, __FILE__, __LINE__);

std::string getOutOfRangeMsg(const int index, const int detect_line_count); 

#endif

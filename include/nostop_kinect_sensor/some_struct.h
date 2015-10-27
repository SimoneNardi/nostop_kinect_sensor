////////////////////////////////////////////////////////////
//	some_struct.h
//	Created on:	27-oct-15
//	Original author: Niko Giovannini Alessandro Faralli
////////////////////////////////////////////////////////////
#ifndef ball_position_struct
#define ball_position_struct
#include <string>

struct ball_position
		{
		  float x;
		  float y;
		  float width;
		  float height;
		};
struct ID{
	    std::string name;
	    std::string front_marker_color;
	    std::string back_marker_color;
	  };

const int ROBOT_NUMBER = 10;
#endif 
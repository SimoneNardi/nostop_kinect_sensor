#ifndef struct_H
#define struct_H
#pragma once


namespace Robotics 
{
	namespace GameTheory
	{
		class ball_position
		{
		public:
		  float x;
		  float y;
		  int height;
		  int width;
		public:
		  ball_position(): x(0), y(0), height(0), width(0) {}
		};

	}
}


#endif // struct_H
#ifndef ball_position_H
#define ball_position_H
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


#endif // ball_position_H
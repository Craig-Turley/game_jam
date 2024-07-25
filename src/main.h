#pragma once

#include <cute.h>


const float RADIUS = 1.0f;

struct Point {
	Cute::v2 position;
	Cute::v2 velocity;
};

struct SoftBody {
	Cute::v2 anchorVertex[4];
	Point points[4];
};

struct GameState
{
	SoftBody body1;
	float k_springForce;
};
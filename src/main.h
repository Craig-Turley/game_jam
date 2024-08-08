#pragma once

#include <cute.h>


const float RADIUS = 1.0f;


struct Point {
	Cute::v2 position;
	Cute::v2 velocity;
  float last_damping;
  Cute::v2 last_velocity;
  Cute::v2 last_position;
  Cute::v2 last_anchor_dist;
};

struct SoftBody {
	Cute::v2 anchorVertex[4];
	Point points[4];
};

struct GameState
{
	SoftBody body1;
	float k_springForce;
	float spring_damping;
	Cute::v2 gravity;
  bool debug_drawTargetShape;
  bool debug_drawCenterOfMass;
};

float calcSoftBodyRotationAngle(SoftBody *body, Cute::v2 centerOfMass);

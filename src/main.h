#pragma once

#include <cute.h>

const float RADIUS = 1.0f;

struct Point {
	Cute::v2 position;
	Cute::v2 velocity;
	float mass;
  float last_damping;
  Cute::v2 last_velocity;
  Cute::v2 last_position;
  Cute::v2 last_anchor_dist;
  Cute::v2 target_point;
};

struct SoftBody {
	Cute::v2 anchorVertex[4];
  Cute::v2 com;
	Point points[4];
  float max_x;
  float max_y;
  float min_x;
  float min_y;
  bool clicked;
};

struct GameState
{
	SoftBody bodies[2];
	SoftBody body1;
	float k_springForce;
	float spring_damping;
	float num_bodies;
  Cute::v2 last_mousedown;
	Cute::v2 gravity;
  Cute::v2 collision_point;
  Cute::v2 farthest_point;
  bool debug_drawTargetShape;
  bool debug_drawCenterOfMass;
  bool debug_drawTargetPointVector;
  bool debug_drawCollisionPoint;
  bool debug_drawBoundingBox;
  bool paused;
  bool done;
  bool nextstep;
};

struct SoftBodyCollision {
	Cute::v2 point;
	Cute::v2 vec;
	Point *c;
	Point *d;
	float distance;
	float u;
  float dist_to_com;
  bool happened;
};

float calcSoftBodyRotationAngle(SoftBody *body, Cute::v2 centerOfMass);
void initScene();
SoftBodyCollision detectCollision(Cute::v2 a, Cute::v2 b, SoftBody *body, int num_points);
void checkBodyCollision(SoftBody *body1, SoftBody *body2, int len1, int len2, float dt);
Cute::v2 findOutsidePoint(SoftBody *body, int num_points, Cute::v2 com);
Cute::v2 calcSoftBodyCenterOfMass(Point *points, int size);

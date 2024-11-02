#pragma once

#include <cute.h>
#include <vector>

const float RADIUS = 1.0f;
const float SCREEN_WIDTH = 640.f;
const float SCREEN_HEIGHT = 480.f;
const float INF = std::numeric_limits<float>::infinity();

struct Point {
	Cute::v2 position;
	Cute::v2 velocity;
  Cute::v2 prev_position;
  Cute::v2 target_point;
  float mass;
  float last_damping;
};

struct BoundingBox {
  float max_x;
  float max_y;
  float min_x;
  float min_y;
};

struct Spring {
  int indexA;
  int indexB;
  float rest_distance;
  float spring_force;

Spring(int idxA, int idxB, float restDist, float springForce)
        : indexA(idxA), indexB(idxB), rest_distance(restDist), spring_force(springForce) {}
};

struct SoftBody {
	Cute::v2 anchorVertex[12];
  Cute::v2 com;
	Point points[12];
  int num_points;
  BoundingBox bounding_box;
  bool clicked;
  std::vector<Spring> springs;
};

struct GasFilledSoftBody {
  Point points[12];
  float restDistances[12];
  int num_points;
  std::vector<Spring> springs;
  float gasForce;
	float spring_force;
	float damping_factor;
  float volume;
};

struct Car {
  SoftBody *car_body;
  GasFilledSoftBody *wheels[2];
  int back_axl_idx[4];
  int front_axl_idx[4];
};

struct GameState
{
	SoftBody bodies[2];
  GasFilledSoftBody gas_bodies[2];
  Car car;
	float k_springForce;
	float spring_damping;
	int num_bodies;
  int num_gas_bodies;
  float gas_force;
  Cute::v2 last_mousedown;
	Cute::v2 gravity;
  bool debug_drawTargetShape;
  bool debug_drawCenterOfMass;
  bool debug_drawCollisionPoint;
  bool debug_drawBoundingBox;
  bool paused;
  bool game_over;
  bool nextstep;
  Cute::v2 axls[4]; //debug purposes`
  float debug_energyCar;
  float debug_energyBackWheel;
  float debug_energyFrontWheel;
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

struct ClosestPoint {
  Cute::v2 point;
  float distance;
  Point *c;
  Point *d;
};

float calcSoftBodyRotationAngle(SoftBody *body, Cute::v2 centerOfMass);
void initScene();
SoftBodyCollision detectCollision(Cute::v2 a, Cute::v2 b, SoftBody *body, int num_points);
void checkBodyCollision(SoftBody *body1, SoftBody *body2, int len1, int len2, float dt);
Cute::v2 findOutsidePoint(SoftBody *body, int num_points, Cute::v2 com);
Cute::v2 calcSoftBodyCenterOfMass(Point *points, int size);

#pragma once

#include <cute.h>
#include <vector>

const float TWO_PI = M_PI * 2;
const float NEG_PI_OVER_ONE_POINT_TWO = (- M_PI) / 1.2f;
const float RADIUS = 1.0f;
const float SCREEN_WIDTH = 640.f;
const float SCREEN_HEIGHT = 480.f;
const float INF = std::numeric_limits<float>::infinity();
const Cute::v2 V2ZERO = cf_v2(0, 0);

struct Point {
	Cute::v2 position;
	Cute::v2 velocity;
  Cute::v2 prev_position;
  float mass;
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
	std::vector<Cute::v2> anchor_vertex; 
  std::vector<Point> points;
  std::vector<Spring> springs;
  Cute::v2 com;
  BoundingBox bounding_box;
  int num_points;
  bool clicked;
};

struct PressureBody {
  std::vector<Point> points;
  std::vector<float> rest_distance;
  std::vector<Spring> springs;
  int num_points;
  float inv_np;
  float max_omega;
  float torque;
  float rotational_velocity;
  float gas_force;
	float spring_force;
	float damping_factor;
  float volume;
  float previous_angle;
};

struct Car {
  SoftBody *car_body;
  PressureBody *wheels[2];
  int back_axl_idx[4];
  int front_axl_idx[4];
};

struct GameState {
  std::vector<SoftBody*> bodies;
  std::vector<PressureBody*> p_bodies;
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

float calc_soft_body_rotation_angle(SoftBody *body, Cute::v2 centerOfMass);
void initScene();
SoftBodyCollision detectCollision(Cute::v2 a, Cute::v2 b, SoftBody *body, int num_points);
void checkBodyCollision(SoftBody *body1, SoftBody *body2, int len1, int len2, float dt);
Cute::v2 findOutsidePoint(SoftBody *body, int num_points, Cute::v2 com);
Cute::v2 calcSoftBodyCenterOfMass(Point *points, int size);
void addSoftBody();

#include "main.h"
#include "debug_draw.h"

#include "cute_math.h"
#include <iostream>

using namespace Cute;

GameState gameState;

SoftBody makeSoftBody() {
	SoftBody body = {};

#if 1
  body.points[0].position = cf_v2(-10, -20);
	body.points[1].position = cf_v2(-10, 10);
	body.points[2].position = cf_v2(10, 10);
	body.points[3].position = cf_v2(10, -10);
#else
  body.points[0].position = cf_v2(-3, -5);
	body.points[1].position = cf_v2(-5, 7);
	body.points[2].position = cf_v2(10, 10);
	body.points[3].position = cf_v2(5, -7);
#endif

  for (int i = 0; i < 4; i++) {
    body.anchorVertex[i] = body.points[i].position;
  }

  return body;
}

inline v2 calcSoftBodyCenterOfMass(SoftBody *body)
{
	v2 com = {};
	for(int i = 0; i < 4; i++) {
		com += body->points[i].position;
	}
	com = com / 4;
	return com;
}

float calcSoftBodyRotationAngle(SoftBody *body, v2 centerOfMass) {
	// F = -kx * dt
	// add F to velocity
	float a = 0.f;
	float b = 0.f;
	for (int i = 0; i < 4; i++) {
		v2 r = body->points[i].position - centerOfMass;
		a += dot(r, body->anchorVertex[i]);
		b += cross(r, body->anchorVertex[i]);
	}
	float angle = -atan2(b, a);
	return angle;
}

void update(float dt) {
	SoftBody *body1 = &gameState.body1;

  // gravity integration
  for(int i = 0; i < 4; i++) {
		Point *p = &body1->points[i];
    p->last_velocity = p->velocity;
    p->last_position = p->position;

    p->velocity += gameState.gravity  * dt;
		p->position += p->velocity * dt;

    float damping = expf(-gameState.spring_damping * dt);
		p->velocity *= damping;
    p->last_damping = damping;
  }

  // collision
	for(int i = 0; i < 4; i++) {
		Point *p = &body1->points[i];
		if (p->position.y < -240.f) { p->position.y = -240.f; }
	}

	// constraints
	v2 com = calcSoftBodyCenterOfMass(body1);
	float angle = calcSoftBodyRotationAngle(body1, com);

  body1->com = com;

  for (int i = 0; i < 4; i++) {
    SinCos rotation = sincos(angle);
    v2 rotatedAnchor = mul(rotation, body1->anchorVertex[i]);
    v2 targetVertex = com + rotatedAnchor;
    v2 x = targetVertex - (com + body1->points[i].position);

    body1->points[i].target_point = x;

    body1->points[i].last_anchor_dist = x;
    body1->points[i].velocity += x * gameState.k_springForce  * dt;
  }
}

void drawSoftBody(SoftBody *body) {

	Point *arr = body->points;
	cf_draw_line(arr[0].position, arr[1].position, .5f);
	cf_draw_line(arr[1].position, arr[2].position, .5f);
	cf_draw_line(arr[2].position, arr[3].position, .5f);
	cf_draw_line(arr[3].position, arr[0].position, .5f);

	draw_push_color(cf_color_red());
	for (int i = 0; i < 4; i++) {
		Point *p = &body->points[i];
		cf_draw_circle2(p->position, RADIUS, 1.0f);
	}
	draw_pop_color();

	v2 com = calcSoftBodyCenterOfMass(body);
	float angle = calcSoftBodyRotationAngle(body, com);

	if (gameState.debug_drawTargetShape){
    draw_push_color(cf_color_green());
    for (int i = 0; i < 4; i++) {
      SinCos rotation = sincos(angle);
      v2 rotatedAnchor = mul(rotation, body->anchorVertex[i]);
      v2 targetVertex = com + rotatedAnchor;

      cf_draw_circle2(targetVertex, RADIUS, 1.0f);
    }
    draw_pop_color();
  }

  if (gameState.debug_drawCenterOfMass) {
    draw_push_color(cf_color_purple());
    v2 com = calcSoftBodyCenterOfMass(body);
    cf_draw_circle2(com, RADIUS, 1.0f);
  }

}

void main_loop(void *udata)
{
	update(CF_DELTA_TIME_FIXED);
}

int main(int argc, char* argv[])
{
	// Create a window with a resolution of 640 x 480.
	int options = APP_OPTIONS_DEFAULT_GFX_CONTEXT | APP_OPTIONS_WINDOW_POS_CENTERED;
	Result result = make_app("Fancy Window Title", 0, 0, 0, 640, 480, options, argv[0]);
	if (is_error(result)) return -1;
	cf_app_init_imgui(false);
	cf_set_fixed_timestep(60);

	gameState.body1 = makeSoftBody();
	gameState.k_springForce = 10.f;
  gameState.spring_damping = 0.8f;
  gameState.gravity = V2(0, -100.f);
  gameState.debug_drawTargetShape = true;
  gameState.debug_drawCenterOfMass = true;

  CF_ASSERT(gameState.k_springForce * CF_DELTA_TIME_FIXED * CF_DELTA_TIME_FIXED < 1.f);
	while (app_is_running())
	{
		app_update(&main_loop);
		// All your game logic and updates go here...
		drawSoftBody(&gameState.body1);
		drawImgui(&gameState);

		app_draw_onto_screen();
	}

	destroy_app();

	return 0;
}

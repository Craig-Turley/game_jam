#include <cute.h>
using namespace Cute;

const float RADIUS = 1.0f;
const float SPRING_FORCE = 100.7f;
struct Point {
	v2 position;
	v2 velocity;
};

struct SoftBody {
  v2 anchorVertex[4];
	Point points[4];
};

SoftBody body1;

SoftBody makeSoftBody() {
	SoftBody body = {};

  body.points[0].position = cf_v2(-10, -10);
	body.points[1].position = cf_v2(-10, 10);
	body.points[2].position = cf_v2(10, 10);
	body.points[3].position = cf_v2(10, -10);

  for (int i = 0; i < 4; i++) {
    body.anchorVertex[i] = body.points[i].position;
  }

  return body;
}

void update(float dt) {
  // gravity integration
  for(int i = 0; i < 4; i++) {
		Point *p = &body1.points[i];
		p->velocity.y += -40.0f * dt;
		p->position += p->velocity * dt;
	}

  // collision
	for(int i = 0; i < 4; i++) {
		Point *p = &body1.points[i];
		if (p->position.y < -240.f) { p->position.y = -240.f; }
	}

  v2 com = {};
  for(int i = 0; i < 4; i++) {
   com += body1.points[i].position;
  }
  com = com / 4;

  // constraints
  // F = -kx * dt
  // add F to velocity
  for (int i = 0; i < 4; i++) {
    v2 targetVertex = com + body1.anchorVertex[i];
    v2 x = targetVertex - body1.points[i].position;
    body1.points[i].velocity += x * SPRING_FORCE  * dt;
  }
}

void drawSoftBody(SoftBody *body) {
	Point *arr = body->points;
	cf_draw_line(arr[0].position, arr[1].position, .5f);
	cf_draw_line(arr[1].position, arr[2].position, .5f);
	cf_draw_line(arr[2].position, arr[3].position, .5f);
	cf_draw_line(arr[3].position, arr[0].position, .5f);

	for (int i = 0; i < 4; i++) {
		Point *p = &body1.points[i];
		cf_draw_circle2(p->position, RADIUS, 1.0f);
	}
}

int main(int argc, char* argv[])
{
	// Create a window with a resolution of 640 x 480.
	int options = APP_OPTIONS_DEFAULT_GFX_CONTEXT | APP_OPTIONS_WINDOW_POS_CENTERED;
	Result result = make_app("Fancy Window Title", 0, 0, 0, 640, 480, options, argv[0]);
	if (is_error(result)) return -1;

  body1 = makeSoftBody();
	while (app_is_running())
	{
		app_update();
		// All your game logic and updates go here...
		update(CF_DELTA_TIME);
		drawSoftBody(&body1);

		app_draw_onto_screen();
	}

	destroy_app();

	return 0;
}

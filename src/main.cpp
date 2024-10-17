#include "main.h"
#include "cute_app.h"
#include "cute_draw.h"
#include "cute_input.h"
#include "debug_draw.h"

#include "cute_math.h"
#include <cmath>
#include <imgui/imgui.h>
#include <iostream>

using namespace Cute;

GameState gameState;

bool inBoundingBox(SoftBody *body, Point *point) {
  float max_x = body->bounding_box.max_x;
  float max_y = body->bounding_box.max_y;
  float min_x = body->bounding_box.min_x;
  float min_y = body->bounding_box.min_y;
  float x = point->position.x;
  float y = point->position.y;
  return (x >= min_x && x <= max_x) && (y >= min_y && y <= max_x);
}

v2 calcSoftBodyCenterOfMass(Point *points, int size) {
  v2 com = {};
  for (int i = 0; i < size; i++) {
    com += points[i].position;
  }
  com = com / size;
  return com;
}

SoftBody makeSoftBody(int offset) {
  SoftBody body = {};

#if 1
  body.points[0].position = cf_v2(-10 + offset, -20);
  body.points[1].position = cf_v2(-10 + offset, 10);
  body.points[2].position = cf_v2(10 + offset, 10);
  body.points[3].position = cf_v2(10 + offset, -10);
#else
  body.points[0].position = cf_v2(-3, -5);
  body.points[1].position = cf_v2(-5, 7);
  body.points[2].position = cf_v2(10, 10);
  body.points[3].position = cf_v2(5, -7);
#endif

  body.points[0].mass = 5;
  body.points[1].mass = 5;
  body.points[2].mass = 5;
  body.points[3].mass = 5;

  body.clicked = false;

  v2 com = calcSoftBodyCenterOfMass(body.points, 4);

   body.bounding_box.max_x = body.points[0].position.x;
   body.bounding_box.max_y = body.points[0].position.y;
   body.bounding_box.min_x = body.points[0].position.x;
   body.bounding_box.min_y = body.points[0].position.y;

  for (int i = 0; i < 4; i++) {
    body.bounding_box.max_x = cf_max(body.bounding_box.max_x, body.points[i].position.x);
    body.bounding_box.max_y = cf_max(body.bounding_box.max_y, body.points[i].position.y);
    body.bounding_box.min_x = cf_max(body.bounding_box.min_x, body.points[i].position.x);
    body.bounding_box.min_y = cf_max(body.bounding_box.min_y, body.points[i].position.y);

    v2 vec = body.points[i].position - com;
    body.anchorVertex[i] = vec;
  }

  body.num_points = 4;

  return body;
}

SoftBody makeCarBody() {
  SoftBody body = {};

  body.points[0].position = cf_v2(-8, 0 ) * 6.5;
  body.points[1].position = cf_v2(-4, 0 ) * 6.5;
  body.points[2].position = cf_v2(-2, 4 ) * 6.5;
  body.points[3].position = cf_v2(0, 4 ) * 6.5;
  body.points[4].position = cf_v2(2, 4 ) * 6.5;
  body.points[5].position = cf_v2(4, 0 ) * 6.5;
  body.points[6].position = cf_v2(6, 0 ) * 6.5;
  body.points[7].position = cf_v2(8, 0 ) * 6.5;
  body.points[8].position = cf_v2(8, -3 ) * 6.5;
  body.points[9].position = cf_v2(4, -4 ) * 6.5;
  body.points[10].position = cf_v2(-4, -4 ) * 6.5;
  body.points[11].position = cf_v2(-8, -3 ) * 6.5;

  body.clicked = false;

  v2 com = calcSoftBodyCenterOfMass(body.points, 12);

  body.bounding_box.max_x = body.points[0].position.x;
  body.bounding_box.max_y = body.points[0].position.y;
  body.bounding_box.min_x = body.points[0].position.x;
  body.bounding_box.min_y = body.points[0].position.y;

  for (int i = 0; i < 12; i++) {
    body.bounding_box.max_x = cf_max(body.bounding_box.max_x, body.points[i].position.x);
    body.bounding_box.max_y = cf_max(body.bounding_box.max_y, body.points[i].position.y);
    body.bounding_box.min_x = cf_max(body.bounding_box.min_x, body.points[i].position.x);
    body.bounding_box.min_y = cf_max(body.bounding_box.min_y, body.points[i].position.y);

    v2 vec = body.points[i].position - com;
    body.anchorVertex[i] = vec;
  }

  body.num_points = 12;

  return body;
}

GasFilledSoftBody makeGasFilledSoftBody(v2 center, float gasForce) {

  GasFilledSoftBody body = {};

	float radius = 7.5;
	int num_points = 8;

	for (int i = 0; i < num_points; i++) {
			float angle = i * 2 * M_PI / num_points;
			body.points[i].position = cf_v2(radius * cos(angle), radius * sin(angle)) + center;
	}

  int n = sizeof(body.points) / sizeof(body.points[0]);

  for (int i = 0; i < 8; i++) {
    int next_idx = (i + 1) % n;

    v2 a = body.points[i].position;
    v2 b = body.points[next_idx].position;

    v2 delta = b - a;
    body.restDistances[i] = cf_len(delta);
  }

  body.gasForce = gasForce;
	body.spring_force = 300.f;
	body.damping_factor = 30.f;
  body.num_points = 8;
  body.bounding_box.max_x = body.points[0].position.x;
  body.bounding_box.max_y = body.points[0].position.y;
  body.bounding_box.min_x = body.points[0].position.x;
  body.bounding_box.min_y = body.points[0].position.y;

  return body;

}

void checkBorderCollisions(Point *p, float dt, float elasticity) {
  if (p->position.y < -(SCREEN_HEIGHT / 2)) {
    p->position.y = -(SCREEN_HEIGHT / 2.f);

    v2 vn = cf_v2(0, -1) * dot(cf_v2(0, -1), p->velocity);
    auto vt = p->velocity - vn;

    vn *= - elasticity;
    vt *= std::exp(- 100 * dt);

    p->velocity = vn + vt;
  }
  if (p->position.x < -(SCREEN_WIDTH / 2)) {
    p->position.x = -(SCREEN_WIDTH / 2);
    p->velocity.x *= -1;
  }
  if (p->position.y > (SCREEN_HEIGHT / 2)) {
    p->position.y = (SCREEN_HEIGHT / 2.f);
    p->velocity.y *= -1;
  }
  if (p->position.x > (SCREEN_WIDTH / 2)) {
    p->position.x = (SCREEN_WIDTH / 2.f);
    p->velocity.x *= -1;
  }

}

float reverse_lerp(v2 a, v2 b, v2 p) {
  return (p.x - a.x) / (b.x - a.x);
}

void handleCollision(Point *a, ClosestPoint closest_point, float restitution) {

  // need to find closest point on the body to the currently tested point
  // move it out and resolve collision between two perfect spheres

  // moving point outside
  v2 correction = closest_point.point - a->position;
  float correction_magnitude = cf_len(correction);
  // moving the ball out and then accounting for radius to perform collision res
  float correction_dist = correction_magnitude + ((RADIUS + RADIUS) / 2.f);
  v2 correction_norm = cf_norm(correction);

  // this gets the ratio of where the point currently is on the line
  float t = reverse_lerp(closest_point.c->position, closest_point.d->position, closest_point.point);

  a->position += correction_norm * correction_dist;
  closest_point.point += correction_norm * -correction_magnitude;
  closest_point.c->position += correction * -correction_magnitude * (1.f - t);
  closest_point.d->position += correction * -correction_magnitude * t;

  // point is now moved outside the body and is now perfectly colliding with
  // our virtual point. now perform the collision between two perfect spheres

  v2 d = closest_point.point - a->position;
  v2 dir = cf_safe_norm(d);

  // skipping the collision check and adjusting since we already know
  // that these two spheres are colliding perfeclty
  // and dont need to be adjusted

  // giving the virtual point an average velocity of the two points that make
  // up the line segment it currently is laying upon
  v2 cd_vel_avg = (closest_point.c->velocity + closest_point.d->velocity) / 2.f;
  float cd_mass_avg = (closest_point.c->mass + closest_point.d->mass) / 2.f;

  float V1 = cf_dot(a->velocity, dir);
  float V2 = cf_dot(cd_vel_avg, dir);

  float M1 = a->mass;
  float M2 = cd_mass_avg;

  float newV1 = (M1 * V1 + M2 * V2 - M2 * (V1 - V2) * restitution) / (M1 + M2);
  float newV2 = (M1 * V1 + M2 * V2 - M1 * (V2 - V1) * restitution) / (M1 + M2);

  a->velocity += dir * (newV1 - V1);
  closest_point.c->velocity += (dir * (newV2 - V2)) * (1.f - t);
  closest_point.d->velocity += (dir * (newV2 - V2)) * t;

}

SoftBodyCollision detectCollision(v2 a, v2 b, SoftBody *body, int num_points) {
  // keep track of closest intersection of collision is detected
  // initialize to the far value so we can always compare less than
  SoftBodyCollision collision = {
    cf_v2(INF, INF),
    cf_v2(INF, INF),
    NULL,
    NULL,
    INF,
    INF,
    INF,
    false
  };
  int collision_count = 0;
  // next check all lines in body
  int n = sizeof(body->points) / sizeof(body->points[0]);

  for (int j = 0; j < num_points; j++) {
    int nextIndex = (j + 1) % n;
    v2 c = body->points[j].position;
    v2 d = body->points[nextIndex].position;
    /* t = Qx/D , s = Qy/D
     * where
     * D = nx*my - ny*mx
     * Qx = my*px - mx*py
     * where
     * nx*t + mx*s = px
     * ny*t + my*s = py
     * where
     * n = b - a
     * m = c - d
     * and p = c - a
     *
     * start from
     * a + t(b-a) = c + s(d-c)
     * */
    v2 n = b - a;
    v2 m = c - d;
    v2 p = c - a;

    float D = n.x * m.y - n.y * m.x;
    float Qx = m.y * p.x - m.x * p.y;
    float Qy = n.x * p.y - n.y * p.x;

    float t = Qx / D;
    float s = Qy / D;

    if (D != 0) {
      if (t >= 0 && t <= 1 && s >= 0 && s <= 1) {
        collision_count++;
      }
    }
  }

  if (collision_count % 2 != 0) {
    collision.happened = true;
  }

  return collision;
}

ClosestPoint findClosestPoint(Point *a, SoftBody *body, v2 com, int num_point) {

  // test every line of the body to find closet point
  // find dc, normalize, dot a and multiply by normal to get point

  ClosestPoint point = {
    cf_v2(INF, INF),
    INF,
    NULL,
    NULL,
  };

  int n = sizeof(body->points) / sizeof(body->points[0]);

  for (int i = 0; i < num_point; i++) {
    int next_idx = (i + 1) % n;

    Point *c = &body->points[i];
    Point *d = &body->points[next_idx];

    //vector from c->d (side being tested)
    v2 dc = d->position - c->position;
    v2 dc_norm = cf_safe_norm(dc);

    // vector from c to point
    v2 cp = a->position - c->position;
    float cp_dot = cf_dot(dc_norm, cp);
    v2 point_on_line = (dc_norm * cp_dot) + c->position; // im adding c to make it relative to 0,0

    float dist_to_point = cf_len(a->position - point_on_line);
    if (dist_to_point < point.distance) {
      point.distance = dist_to_point;
      point.point = point_on_line;
      point.c = c;
      point.d = d;
    }

  }

  return point;

}

void checkBodyCollision(Point *point, SoftBody *body2, int num_points1, int num_points2) {

  // find com for body2
  v2 body2_com = calcSoftBodyCenterOfMass(body2->points, num_points2);

  for (int i = 0; i < num_points1; i++) {
    // current point being tested
    if (!inBoundingBox(body2, point)) {
      continue;
    }
    v2 a = point->position;
    // point outside bounding box axis aligned(x) with current
    // point being tested. added 5 as a buffer to get outside the box
    v2 b = cf_v2(body2->bounding_box.max_x + 5, a.y);
    SoftBodyCollision collision = detectCollision(a, b, body2, num_points2);

    if (collision.happened) {
      ClosestPoint closest_point = findClosestPoint(point, body2, body2_com, num_points2);
      gameState.collision_point = closest_point.point;
      //gameState.debug_drawCollisionPoint = true;
      //gameState.paused = true;
      handleCollision(point, closest_point,  0.5f);
    }
  }
}

float calcSoftBodyRotationAngle(SoftBody *body, v2 com) {
  // F = -kx * dt
  // add F to velocity
  float a = 0.f;
  float b = 0.f;
  for (int i = 0; i < body->num_points; i++) {
    v2 r = body->points[i].position - com;
    a += dot(r, body->anchorVertex[i]);
    b += cross(r, body->anchorVertex[i]);
  }
  float angle = -atan2(b, a);
  return angle;
}

v2 rotate(v2 *anchorVertex, float angle) {
  SinCos rotation = sincos(angle);
  v2 rotatedAnchor = mul(rotation, *anchorVertex);
  return rotatedAnchor;
}

v2 calcSoftBodyAverageVelocity(Point *points, int length) {
  v2 vel = {};
  for (int i = 0; i < length; i++) {
    vel += points[i].velocity;
  }
  vel = vel / float(length);
  return vel;
}

v2 calculateWheelAxel(GasFilledSoftBody *wheel) {
  v2 wheel_axl = cf_v2(0, 0);
  int cnt = 0;

  for (int i = 0; i < wheel->num_points; i += 2) {
    wheel_axl += wheel->points[i].position;

    cnt++;
  }

  wheel_axl /= (float)cnt;

  return wheel_axl;
}

void checkMouseDown(float dt) {
  if (cf_mouse_down(MOUSE_BUTTON_LEFT)) {

    float mouse_x = cf_mouse_x() - (SCREEN_WIDTH / 2);
    float mouse_y = (cf_mouse_y() - (SCREEN_HEIGHT / 2)) * -1;
    v2 mouse_center = V2(mouse_x, mouse_y);

    v2 distance = mouse_center - gameState.last_mousedown;
    v2 velocity = distance / dt;

    for (int i = 0; i < gameState.num_bodies; i++) {

      SoftBody *body = &gameState.bodies[i];

      // find com for body
      v2 com = calcSoftBodyCenterOfMass(body->points, body->num_points);

      // initialize longest point
      v2 farthest_point = com - body->points[0].position;
      float farthest_point_len = cf_len(farthest_point);
      for (int i = 0; i < body->num_points; i++) {
        v2 cur_point = com - body->points[i].position;
        float cur_point_len = cf_len(cur_point);
        if (cur_point_len > farthest_point_len) {
          farthest_point = cur_point;
          farthest_point_len = cur_point_len;
        }
      }

      // we now have a point we know is the farthest from the COM and can double
      // it to get a point outside the shape
      v2 a = mouse_center;
      v2 b = (farthest_point + com) * 2;

      // all points are now relative to 0,0
      int n = sizeof(body->points) / sizeof(body->points[i]);

      int collision_count = 0;
      for (int j = 0; j < body->num_points; j++) {
        int nextIndex = (j + 1) % n;
        v2 c = body->points[j].position;
        v2 d = body->points[nextIndex].position;
        float t_top = (d.x - c.x) * (a.y - c.y) - (d.y - c.y) * (a.x - c.x);
        float u_top = (c.y - a.y) * (a.x - b.x) - (c.x - a.x) * (a.y - b.y);
        float bottom = (d.y - c.y) * (b.x - a.x) - (d.x - c.x) * (b.y - a.y);
        float t = t_top / bottom;
        float u = u_top / bottom;

        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
          collision_count++;
        }
      }

      if (collision_count % 2 != 0) {
        for (int i = 0; i < 4; i++) {
          body->points[i].position = mouse_center + body->anchorVertex[i];
        }
      } else {
        body->clicked = false;
      }

      gameState.last_mousedown = mouse_center;
    }
  }
}

void checkInputs(float dt) {
  for (int i = 0; i < 2; i++) {
    GasFilledSoftBody *wheel = &gameState.gas_bodies[i];
    if (cf_key_down(CF_KEY_D)){

      v2 axel = calculateWheelAxel(wheel);

      for (int i = 0; i < 8; i++) {
        Point *p = &wheel->points[i];
        v2 r = cf_safe_norm(p->position - axel);

        v2 dir = cf_v2(r.y, -r.x);
        p->position += dir;
      }
    }


    if (cf_key_down(CF_KEY_A)){

      v2 axel = calculateWheelAxel(wheel);

      for (int i = 0; i < 8; i++) {
        Point *p = &wheel->points[i];
        v2 r = cf_safe_norm(p->position - axel);

        v2 dir = cf_v2(-r.y, r.x);
        p->position += dir;
      }
    }
  }
}

void updateSoftBodies(float dt) {

  for (int i = 0; i < gameState.num_bodies; i++) {

    SoftBody *body = &gameState.bodies[i];

    checkMouseDown(dt);
    checkInputs(dt);

    float max_x = body->points[0].position.x;
    float max_y = body->points[0].position.y;
    float min_x = body->points[0].position.x;
    float min_y = body->points[0].position.y;

    // gravity integration
    for (int i = 0; i < body->num_points; i++) {
      Point *p = &body->points[i];

      // add grav if its not clicked
      if (!body->clicked) {
        p->velocity += gameState.gravity * dt;
        p->position += p->velocity * dt;
      }

      max_x = cf_max(max_x, body->points[i].position.x);
      max_y = cf_max(max_y, body->points[i].position.y);
      min_x = cf_min(min_x, body->points[i].position.x);
      min_y = cf_min(min_y, body->points[i].position.y);
    }

    // set max and min points for bounding box

    body->bounding_box.max_x = max_x;
    body->bounding_box.max_y = max_y;
    body->bounding_box.min_x = min_x;
    body->bounding_box.min_y = min_y;

    v2 com = calcSoftBodyCenterOfMass(body->points, body->num_points);

    // collision
    for (int i = 0; i < gameState.num_bodies; i++) {
      SoftBody *body = &gameState.bodies[i];

      for (int i = 0; i < body->num_points; i++) {
        Point *point = &body->points[i];
        checkBorderCollisions(point, dt, -0.5);
      }

      for (int j = 0; j < gameState.num_bodies; j++) {
        if (j == i) {
          // skip self collision
          continue;
        }
        SoftBody *body2 = &gameState.bodies[j];

        for (int k = 0; k < body->num_points; k++) {
          Point *point = &body->points[k];
          checkBodyCollision(point, body2, body->num_points, body2->num_points);
        }
      }
    }

    // constraints
    float angle = calcSoftBodyRotationAngle(body, com);
    v2 avg_vel = calcSoftBodyAverageVelocity(body->points, body->num_points);

    for (int i = 0; i < body->num_points; i++) {

      Point *point = &body->points[i];
      v2 targetVertex = com + rotate(&body->anchorVertex[i], angle);
      v2 x = targetVertex - point->position;
      v2 target_vertex_corrected = body->anchorVertex[i] + com;

      // spring force
      v2 spring_force = x * gameState.k_springForce * dt;
      point->velocity += spring_force;
    }

    // bascially just damping between every edge treating it as a spring

    int n = sizeof(body->points) / sizeof(body->points[0]);

    for (int i = 0; i < body->num_points; i++) {

      Point *point = &body->points[i];
      // this gets the next point to calc rest distance
      int next_idx = (i + 1) % n;
      Point *next_point = &body->points[next_idx];
      v2 rest_distance_vec =
          body->anchorVertex[next_idx] - body->anchorVertex[i];
      float rest_distance = cf_len(rest_distance_vec);

      v2 p0 = point->position;
      v2 p1 = next_point->position;
      v2 v0 = point->velocity;
      v2 v1 = next_point->velocity;

      v2 delta = p1 - p0;
      v2 direction = cf_safe_norm(delta);

      float vrel = dot(v1 - v0, direction);
      float damping_force = expf(-gameState.spring_damping * dt);
      float new_vrel = vrel * damping_force;
      float vrel_delta = new_vrel - vrel;

      point->velocity -= direction * (vrel_delta / 2.0);
      next_point->velocity += direction * (vrel_delta / 2.0);
    }
  }
}

float getVolume(GasFilledSoftBody *body, int num_points) {
	float volume = 0;

	int n = sizeof(body->points) / sizeof(body->points[0]);

	for (int i = 0; i < num_points; i++) {
		float x1 = body->points[i].position.x;
		float y1 = body->points[i].position.y;
		float x2 = body->points[(i + 1) % n].position.x;
		float y2 = body->points[(i + 1) % n].position.y;

		float r12d = sqrt(
      (x1 - x2) * (x1 - x2) +
      (y1 - y2) * (y1 - y2)
    );

    v2 norm = cf_safe_norm(body->points[(i + 1) % n].position - body->points[i].position);

    volume += 0.5f * fabs(x1 - x2) * fabs(norm.x) * r12d;
	}

	return volume;

}

void updateGasFilledSoftBodies(float dt) {

  for (int i = 0; i < gameState.num_gas_bodies; i++) {

    // integrate gravity
    GasFilledSoftBody *body = &gameState.gas_bodies[i];

    float max_x = body->points[0].position.x;
    float max_y = body->points[0].position.y;
    float min_x = body->points[0].position.x;
    float min_y = body->points[0].position.y;

    for (int pointIdx = 0; pointIdx < 8; pointIdx++) {
      Point *p = &body->points[pointIdx];

      p->velocity += gameState.gravity * dt;
      p->position += p->velocity * dt;

      max_x = cf_max(max_x, body->points[i].position.x);
      max_y = cf_max(max_y, body->points[i].position.y);
      min_x = cf_min(min_x, body->points[i].position.x);
      min_y = cf_min(min_y, body->points[i].position.y);
    }

    // set max and min points for bounding box

    body->bounding_box.max_x = max_x;
    body->bounding_box.max_y = max_y;
    body->bounding_box.min_x = min_x;
    body->bounding_box.min_y = min_y;


    //check collision
    for (int pointIdx = 0; pointIdx < 8; pointIdx++) {
      checkBorderCollisions(&body->points[pointIdx], dt, 0.5);
    }

    // satisfy contraints

		v2 com = calcSoftBodyCenterOfMass(body->points, 8);
		float volume = getVolume(body, 8);
		body->volume = volume;

    int n = sizeof(body->points) / sizeof(body->points[0]);


    for (int i = 0; i < 8; i++) {
      float x1 = body->points[i].position.x;
      float y1 = body->points[i].position.y;
      float x2 = body->points[(i + 1) % n].position.x;
      float y2 = body->points[(i + 1) % n].position.y;

      float r12d = sqrt(
        (x1 - x2) * (x1 - x2) +
        (y1 - y2) * (y1 - y2)
      );

      float pressurev = r12d * body->gasForce * (1.0f / volume);

      v2 rest_distance = cf_safe_norm(body->points[(i + 1) % n].position - body->points[i].position) * body->restDistances[i];
      v2 norm = cf_safe_norm(rest_distance);
      v2 dir = cf_v2(norm.y, - norm.x);

      body->points[i].velocity += dir * pressurev;
      body->points[(i + 1) % n].velocity += dir * pressurev;

    }

    // rest distance idx
    for (int rdIdx = 0; rdIdx < 8; rdIdx++) {
      int nextIdx = (rdIdx + 1) % n;
      float rd = body->restDistances[rdIdx];

      Point *a = &body->points[rdIdx];
      Point *b = &body->points[nextIdx];

      v2 delta = b->position - a->position;
      float distance = cf_len(delta);
      v2 direction = delta / distance;

      v2 required_delta = delta * (rd / distance);
      v2 force = (required_delta - delta) * body->spring_force;

      a->velocity -= force * dt;
      b->velocity += force * dt;

      float vrel = dot(b->velocity - a->velocity, direction);
      float damping_factor = exp(-body->damping_factor * dt);
      float new_vrel = vrel * damping_factor;
      float vrel_delta = new_vrel - vrel;

      a->velocity -= direction * vrel_delta / 2.0;
      b->velocity += direction * vrel_delta / 2.0;
    }

  }

}


void updateCar(float dt) {
  Car car = gameState.car;

  int back_axl_idx[4] = {10, 11, 0, 1};
  int front_axl_idx[4] = {9, 8, 7, 5};

  SoftBody *car_body = car.car_body;

  v2 com = calcSoftBodyCenterOfMass(car.car_body->points, car.car_body->num_points);

  float weights[4] = {3, 2, 1, 1};
  float weights_sum = 7;

  v2 back_axl = cf_v2(0, 0);
  v2 front_axl = cf_v2(0, 0);
  for (int i = 0; i < 4; i++) {
    float weight = weights[i];
    int back_index = back_axl_idx[i];
    int front_index = front_axl_idx[i];

    back_axl += car_body->points[back_index].position * weight;
    front_axl += car_body->points[front_index].position * weight;
  }

  back_axl /= weights_sum;
  front_axl /= weights_sum;

  // lets get the axl for the wheels now;

  GasFilledSoftBody *back_wheel = car.wheels[0];
  GasFilledSoftBody *front_wheel = car.wheels[1];

  v2 back_wheel_axl = calculateWheelAxel(back_wheel);
  v2 front_wheel_axl = calculateWheelAxel(front_wheel);

  gameState.axls[0] = back_axl;
  gameState.axls[1] = front_axl;
  gameState.axls[2] = back_wheel_axl;
  gameState.axls[3] = front_wheel_axl;
/*
  if (back_wheel_axl.x != back_axl.x && back_wheel_axl.y != back_axl.y) {
    v2 cor = back_axl - back_wheel_axl;
    for (int i = 0; i < 4; i++) {
      int car_axl = back_axl_idx[i];
      int wheel_axl = i * 2 - 1;

      car.car_body->points[car_axl].position -= cor / 2.f;
      car.wheels[0]->points[wheel_axl].position += cor / 2.f;
    }
  }

  if (front_wheel_axl.x != front_axl.x && front_wheel_axl.y != front_axl.y) {
    v2 cor = front_axl - front_wheel_axl;
    for (int i = 0; i < 4; i++) {
      int car_axl = front_axl_idx[i];
      int wheel_axl = i * 2 - 1;

      car.car_body->points[car_axl].position -= cor / 2.f;
      car.wheels[1]->points[wheel_axl].position += cor / 2.f;
    }
  }
  */
}

void update(float dt) {

  updateSoftBodies(dt);

  updateGasFilledSoftBodies(dt);

  updateCar(dt);
}

void drawSoftBody(SoftBody *body) {

  int n = sizeof(body->points) / sizeof(body->points[0]);

  for (int i = 0; i < body->num_points; i++) {
    v2 a = body->points[i].position;
    v2 b = body->points[(i + 1) % n].position;

    cf_draw_line(a, b, 0.5f);
  }

  draw_push_color(cf_color_red());
  for (int i = 0; i < body->num_points; i++) {
    Point *p = &body->points[i];
    cf_draw_circle2(p->position, RADIUS, 1.0f);
  }

  if (gameState.debug_drawBoundingBox) {
    v2 t_left = cf_v2(body->bounding_box.min_x, body->bounding_box.max_y);
    v2 b_left = cf_v2(body->bounding_box.min_x, body->bounding_box.min_y);
    v2 t_right = cf_v2(body->bounding_box.max_x, body->bounding_box.max_y);
    v2 b_right = cf_v2(body->bounding_box.max_x, body->bounding_box.min_y);
    cf_draw_line(t_left, t_right, .5f);
    cf_draw_line(t_right, b_right, .5f);
    cf_draw_line(b_right, b_left, .5f);
    cf_draw_line(b_left, t_left, .5f);
  }

  draw_pop_color();

  v2 com = calcSoftBodyCenterOfMass(body->points, body->num_points);
  float angle = calcSoftBodyRotationAngle(body, com);

  if (gameState.debug_drawTargetShape) {
    draw_push_color(cf_color_green());
    for (int i = 0; i < body->num_points; i++) {
      SinCos rotation = sincos(angle);
      v2 rotatedAnchor = mul(rotation, body->anchorVertex[i]);
      v2 targetVertex = com + rotatedAnchor;

      cf_draw_circle2(targetVertex, RADIUS, 1.0f);
    }
    draw_pop_color();
  }

  if (gameState.debug_drawCenterOfMass) {
    draw_push_color(cf_color_purple());
    cf_draw_circle2(com, RADIUS, 1.0f);
    draw_pop_color();
  }

  if (gameState.debug_drawCollisionPoint) {
    draw_push_color(cf_color_orange());
    cf_draw_circle2(gameState.collision_point, RADIUS, 1.f);
    cf_draw_pop_color();
    cf_draw_circle2(gameState.farthest_point, RADIUS, 1.f);
  }

  draw_pop_color();

  draw_push_color(cf_color_orange());
  for (int i = 0; i < 4; i++) {
    cf_draw_circle2(gameState.axls[i], RADIUS, 1.f);
  }
  draw_pop_color();
}

void drawGasFilledSoftBody(GasFilledSoftBody *body) {

  // draw lines
  int n = sizeof(body->points) / sizeof(body->points[0]);

  for (int i = 0; i < 8; i++) {
    int nextIdx = (i + 1) % n;

    cf_draw_line(body->points[i].position, body->points[nextIdx].position , 0.5f);
  }

  // draw points
  draw_push_color(cf_color_red());
  for (int i = 0; i < 8; i++) {
    cf_draw_circle2(body->points[i].position, RADIUS, 1.0f);
  }
  draw_pop_color();

}

void main_loop(void *udata) {
  update(CF_DELTA_TIME_FIXED);
}

void initScene() {
  gameState.num_bodies = 1;
  gameState.bodies[0] = makeCarBody();
  gameState.num_gas_bodies = 2;
  gameState.gas_bodies[0] = makeGasFilledSoftBody(cf_v2(-20, 0), 300.0);
  gameState.gas_bodies[1] = makeGasFilledSoftBody(cf_v2(20, 0), 300.0);
  gameState.k_springForce = 100.f;
  gameState.spring_damping = 10.f;
  gameState.gravity = V2(0, -9.8f);
  gameState.debug_drawTargetShape = true;
  gameState.debug_drawCenterOfMass = true;
  gameState.last_mousedown = V2(0.0, 0.0);

  gameState.car = {};
  gameState.car.car_body = &gameState.bodies[0];
  gameState.car.wheels[0] = &gameState.gas_bodies[0];
  gameState.car.wheels[1] = &gameState.gas_bodies[1];
}

int main(int argc, char *argv[]) {
  // Create a window with a resolution of 640 x 480.
  int options = APP_OPTIONS_WINDOW_POS_CENTERED;
  Result result = make_app("Fancy Window Title", 0, 0, 0, SCREEN_WIDTH,
                           SCREEN_HEIGHT, options, argv[0]);
  if (is_error(result))
    return -1;
  cf_app_init_imgui(false);
  cf_set_fixed_timestep(60);

  initScene();

  CF_ASSERT(gameState.k_springForce * CF_DELTA_TIME_FIXED *
                CF_DELTA_TIME_FIXED <
            1.f);
  while (app_is_running()) {
    app_update(&main_loop);
    // All your game logic and updates go here...
    for (int i = 0; i < gameState.num_bodies; i++) {
      SoftBody *body = &gameState.bodies[i];
      drawSoftBody(body);
    }

    for (int i = 0; i < gameState.num_gas_bodies; i++) {
      GasFilledSoftBody *body = &gameState.gas_bodies[i];
      drawGasFilledSoftBody(body);
    }

    drawImgui(&gameState);

    app_draw_onto_screen(true);

    if (gameState.game_over) {
      break;
    }
  }

  destroy_app();

  return 0;
}

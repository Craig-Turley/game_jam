#include "main.h"
#include "cute_app.h"
#include "debug_draw.h"

#include "cute_math.h"
#include <imgui/imgui.h>
#include <iostream>

using namespace Cute;

const float SCREEN_WIDTH = 640.f;
const float SCREEN_HEIGHT = 480.f;
const float INF = std::numeric_limits<float>::infinity();

GameState gameState;

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

  body.max_x = 0;
  body.max_y = 0;
  body.clicked = false;

  v2 com = calcSoftBodyCenterOfMass(body.points, 4);

  for (int i = 0; i < 4; i++) {
    v2 vec = body.points[i].position - com;
    body.anchorVertex[i] = vec;
    body.max_x = cf_max(vec.x, body.max_x);
    body.max_y = cf_max(vec.y, body.max_y);
  }

  return body;
}

void checkBorderCollisions(Point *p) {
  if (p->position.y < -(SCREEN_HEIGHT / 2)) {
    p->position.y = -(SCREEN_HEIGHT / 2.f);
    p->velocity.y *= -1;
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

void handleCollision(Point *a, SoftBodyCollision collision, float restitution, float dt) {
  // move point out of body
  v2 dir = cf_safe_norm(collision.vec);
  float d = cf_len(dir);

  float correction = d + RADIUS + RADIUS / 2.f;
  a->position += dir * correction;
  collision.c->position += dir * -correction * (1.f - collision.u);
  collision.d->position += dir * -correction * collision.u;

  dir = cf_safe_norm(cf_lerp_v2(collision.c->position, collision.d->position, collision.u) - a->position);

  float M1 = a->mass;
  float V1 = cf_dot(a->velocity, dir);

  float M2 = (collision.c->mass + collision.d->mass) / 2.f;
  float V2 = cf_dot((collision.c->velocity + collision.d->velocity) / 2.f, dir);

  float v1_prime =
      (V1 * M1 + V2 * M2 - (V1 - V2) * M2 * restitution) / (M1 + M2);
  float v2_prime =
      (V1 * M1 + V2 * M2 - (V2 - V1) * M1 * restitution) / (M1 + M2);

  a->velocity += dir * (v1_prime - V1);
  collision.c->velocity += dir * ((v2_prime * (1.f - collision.u)) - V2);
  collision.d->velocity += dir * (v2_prime * collision.u - V2);
}
/*
const handleBallCollision = (ball1, ball2, restitution) => {
  let dir = new Vector2();
  dir.subtractVectors(ball2.pos, ball1.pos);
  let d = dir.length();
  // no collision occured
  if (d == 0.0 || d > ball1.radius + ball2.radius)
    return;

  dir.scale(1.0 / d);

  let corr = (ball1.radius + ball2.radius - d) / 2.0;
  ball1.pos.add(dir, -corr);
  ball2.pos.add(dir, corr);

  let v1 = ball1.vel.dot(dir);
  let v2 = ball2.vel.dot(dir);

  let m1 = ball1.mass;
  let m2 = ball2.mass;

  let newV1 = (m1 * v1 + m2 * v2 - m2 * (v1 - v2) * restitution) / (m1 + m2);
  let newV2 = (m1 * v1 + m2 * v2 - m1 * (v2 - v1) * restitution) / (m1 + m2);

  ball1.vel.add(dir, newV1 - v1);
  ball2.vel.add(dir, newV2 - v2);
};
*/

v2 findOutsidePoint(SoftBody *body, int num_points, v2 com) {
  v2 farthest_point = body->points[0].position - com;
  float farthest_point_len = cf_len(farthest_point);
  for (int i = 0; i < num_points; i++) {
    v2 cur_point = body->points[i].position - com;
    float cur_point_len = cf_len(cur_point);
    if (cur_point_len > farthest_point_len) {
      farthest_point = cur_point;
      farthest_point_len = cur_point_len;
    }
  }

  v2 outside_point = farthest_point = (farthest_point * 2) + com;

  return farthest_point;
}

SoftBodyCollision detectCollision(v2 a, v2 b, v2 com, SoftBody *body, int num_points) {
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
    float t_top = (d.x - c.x) * (a.y - c.y) - (d.y - c.y) * (a.x - c.x);
    float u_top = (c.y - a.y) * (a.x - b.x) - (c.x - a.x) * (a.y - b.y);
    float bottom = (d.y - c.y) * (b.x - a.x) - (d.x - c.x) * (b.y - a.y);
    float t = t_top / bottom;
    float u = u_top / bottom;

    if (bottom != 0) {
      if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        collision_count++;
        // since c and d are relative to 0, this point is also relative to 0
        // we find the point on the line of the sofy body being tested against
        // not the softbody of the individua point being tested
        v2 point_on_line = cf_lerp_v2(c, d, u);
        v2 dist_to_current_point_vec = point_on_line - a;
        float dist_to_current_point = cf_len(dist_to_current_point_vec);
        // keeping track of the closest point to the center of mass to use for collision
        float dist_to_com = cf_len(com - point_on_line);
        if (dist_to_com < collision.distance) {
          collision.point = point_on_line;
          collision.vec = dist_to_current_point_vec;
          collision.distance = dist_to_com;
          collision.c = &body->points[j];
          collision.d = &body->points[nextIndex];
          collision.u = u;
        }
      }
    }
  }

  if (collision_count % 2 != 0) {
    collision.happened = true;
  }

  return collision;
}

void checkBodyCollision(SoftBody *body1, SoftBody *body2, int len1, int len2,
                        float dt) {

  // find com for body1
  v2 body1_com = calcSoftBodyCenterOfMass(body1->points, 4);

  // find com for body2
  v2 body2_com = calcSoftBodyCenterOfMass(body2->points, 4);

  // initialize longest point
  v2 farthest_point = findOutsidePoint(body2, 4, body2_com);

  // all points are now relative to 0,0

  for (int i = 0; i < len1; i++) {
    // current point being tested
    Point *a_point = &body1->points[i];
    v2 a = body1->points[i].position;
    // farthest point from the COM and double it to get a point outside the
    // shape
    v2 b = farthest_point;
    SoftBodyCollision collision = detectCollision(a, b, body1_com, body2, len2);


    if (collision.happened) {
      //gameState.paused = true;
      //gameState.debug_drawCollisionPoint = true;
      gameState.collision_point = a;
      gameState.farthest_point = farthest_point;
      handleCollision(a_point, collision, .54f, dt);
    }

  }
}

float calcSoftBodyRotationAngle(SoftBody *body, v2 com) {
  // F = -kx * dt
  // add F to velocity
  float a = 0.f;
  float b = 0.f;
  for (int i = 0; i < 4; i++) {
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
  for (int i = 0; i < 4; i++) {
    vel += points[i].velocity;
  }
  vel = vel / float(length);
  return vel;
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
      v2 com = calcSoftBodyCenterOfMass(body->points, 4);

      // initialize longest point
      v2 farthest_point = com - body->points[0].position;
      float farthest_point_len = cf_len(farthest_point);
      for (int i = 0; i < 4; i++) {
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
      for (int j = 0; j < 4; j++) {
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

void update(float dt) {

  for (int i = 0; i < gameState.num_bodies; i++) {

    SoftBody *body = &gameState.bodies[i];

    checkMouseDown(dt);

    // gravity integration
    for (int i = 0; i < 4; i++) {
      Point *p = &body->points[i];
      p->last_velocity = p->velocity;
      p->last_position = p->position;

      // add grav if its not clicked
      if (!body->clicked) {
        p->velocity += gameState.gravity * dt;
        p->position += p->velocity * dt;
      }
    }

    v2 com = calcSoftBodyCenterOfMass(body->points, 4);

    // collision
    for (int i = 0; i < gameState.num_bodies; i++) {
      SoftBody *body = &gameState.bodies[i];

      for (int i = 0; i < 4; i++) {
        Point *point = &body->points[i];
        checkBorderCollisions(point);
      }

      for (int j = 0; j < gameState.num_bodies; j++) {
        if (j == i) {
          // skip self collision
          continue;
        }
        SoftBody *body2 = &gameState.bodies[j];
        checkBodyCollision(body, body2, 4, 4, dt);
      }
    }

    // constraints
    float angle = calcSoftBodyRotationAngle(body, com);
    v2 avg_vel = calcSoftBodyAverageVelocity(body->points, 4);

    for (int i = 0; i < 4; i++) {

      Point *point = &body->points[i];
      v2 targetVertex = com + rotate(&body->anchorVertex[i], angle);
      v2 x = targetVertex - point->position;
      v2 target_vertex_corrected = body->anchorVertex[i] + com;

      // for debug log
      point->target_point = targetVertex;
      point->last_anchor_dist = x;

      // spring force
      v2 spring_force = x * gameState.k_springForce * dt;
      point->velocity += spring_force;
    }

    // damping

    // bascially just damping between every edge treating it as a spring

    int n = sizeof(body->points) / sizeof(body->points[0]);

    for (int i = 0; i < 4; i++) {

      Point *point = &body->points[i];
      // this gets the next point to calc rest distance
      int next_idx = (i + 1) % n;
      Point *next_point = &body->points[next_idx];
      v2 rest_distance_vec =
          body->anchorVertex[next_idx] - body->anchorVertex[i];
      float rest_distance = cf_len(rest_distance_vec);

      v2 p0 = point->position - com;
      v2 p1 = next_point->position - com;
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

  v2 com = calcSoftBodyCenterOfMass(arr, 4);
  float angle = calcSoftBodyRotationAngle(body, com);

  if (gameState.debug_drawTargetShape) {
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
    cf_draw_circle2(com, RADIUS, 1.0f);
    draw_pop_color();
  }

  if (gameState.debug_drawTargetPointVector) {
    draw_push_color(cf_color_orange());
    for (int i = 0; i < 4; i++) {
      draw_push_color(cf_color_magenta());
      cf_draw_circle2(arr[i].position, RADIUS, 1.f);
      cf_draw_circle2(arr[i].target_point, RADIUS, 1.f);
      draw_pop_color();
      cf_draw_line(arr[i].position, arr[i].target_point, .5f);
    }
  }

  if (gameState.debug_drawCollisionPoint) {
    draw_push_color(cf_color_orange());
    cf_draw_circle2(gameState.collision_point, RADIUS, 1.f);
    cf_draw_pop_color();
    draw_push_color(cf_color_yellow());
    cf_draw_circle2(gameState.farthest_point, RADIUS, 1.f);
    draw_pop_color();
    draw_push_color(cf_color_red());
    cf_draw_line(gameState.collision_point, gameState.farthest_point, .5f);
    draw_pop_color();
  }

  draw_pop_color();
}

void main_loop(void *udata) {
  if (gameState.paused)
    return;
  update(CF_DELTA_TIME_FIXED);
}

void initScene() {
  gameState.num_bodies = 2;
  for (int i = 0; i < gameState.num_bodies; i++) {
    gameState.bodies[i] = makeSoftBody(i * 50);
  }
  gameState.k_springForce = 100.f;
  gameState.spring_damping = 10.f;
  gameState.gravity = V2(0, -9.8f);
  gameState.debug_drawTargetShape = true;
  gameState.debug_drawCenterOfMass = true;
  gameState.last_mousedown = V2(0.0, 0.0);
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
    drawImgui(&gameState);

    app_draw_onto_screen(true);

    if (gameState.done) {
      break;
    }
  }

  destroy_app();

  return 0;
}

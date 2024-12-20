#include "main.h"
#include "cute_app.h"
#include "cute_color.h"
#include "cute_draw.h"
#include "cute_input.h"
#include "debug_draw.h"

#include "cute_math.h"
#include <cmath>
#include <imgui/imgui.h>
#include <iostream>

using namespace Cute;

GameState gameState;

Point MakePoint(v2 pos) {
  Point p;

  p.position = pos;
  p.velocity = V2ZERO; 
  p.prev_position = pos;
  p.mass = 1;

  return p;
}

bool inBoundingBox(SoftBody *body, Point *point) {
  float max_x = body->bounding_box.max_x;
  float max_y = body->bounding_box.max_y;
  float min_x = body->bounding_box.min_x;
  float min_y = body->bounding_box.min_y;
  float x = point->position.x;
  float y = point->position.y;
  return (x >= min_x && x <= max_x) && (y >= min_y && y <= max_x);
}

v2 calc_soft_body_center_of_mass(Point *points, int size) {
  v2 com = {};
  for (int i = 0; i < size; i++) {
    com += points[i].position;
  }
  com = com / size;
  return com;
}

SoftBody makeSoftBody(v2 center) {
  SoftBody body = {};

  body.points.push_back(MakePoint(cf_v2(-10, -10) + center));
  body.points.push_back(MakePoint(cf_v2(-10, 10) + center));
  body.points.push_back(MakePoint(cf_v2(10, 10) + center));
  body.points.push_back(MakePoint(cf_v2(10, -10) + center));
  
  body.clicked = false;

  v2 com = calc_soft_body_center_of_mass(body.points.data(), 4);

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
    body.anchor_vertex.push_back(vec);
  }

  body.num_points = 4;

  return body;
}


SoftBody MakeCarBody(v2 center) {
  SoftBody body = {};

  body.points.push_back(MakePoint(cf_v2(-8, 0 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(-4, 0 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(-2, 4 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(0, 4 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(2, 4 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(4, 0 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(6, 0 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(8, 0 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(8, -3 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(4, -4 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(-4, -4 ) * 6.5 + center));
  body.points.push_back(MakePoint(cf_v2(-8, -3 ) * 6.5 + center));

  body.clicked = false;

  v2 com = calc_soft_body_center_of_mass(body.points.data(), 12);

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
    body.anchor_vertex.push_back(vec);
  }

  body.num_points = 12;

  body.springs.emplace_back(
      0, 10,
      cf_len(body.points[10].position - body.points[0].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      1, 11,
      cf_len(body.points[11].position - body.points[1].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      2, 9,
      cf_len(body.points[9].position - body.points[2].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      4, 10,
      cf_len(body.points[10].position - body.points[4].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      5, 8,
      cf_len(body.points[8].position - body.points[5].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      7, 9,
      cf_len(body.points[9].position - body.points[7].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      3, 10,
      cf_len(body.points[10].position - body.points[3].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      3, 9,
      cf_len(body.points[9].position - body.points[3].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      6, 9,
      cf_len(body.points[9].position - body.points[6].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      6, 8,
      cf_len(body.points[8].position - body.points[6].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      1, 9,
      cf_len(body.points[9].position - body.points[1].position),
      gameState.k_springForce
  );
  body.springs.emplace_back(
      5, 10,
      cf_len(body.points[10].position - body.points[5].position),
      gameState.k_springForce
  );

  return body;
}

PressureBody MakePressureBody(v2 center, float gas_force, int num_points) {

  PressureBody body = {};

  // todo incorporate this into constructor
  float radius = 5.f;
  float spring_force = 1200.f;

  // making a circle
  for (int i = 0; i < num_points; i++) {
    float theta = (float)i * 2.0 * M_PI / (float)num_points;
    body.points.push_back(MakePoint((cf_v2(cos(theta), sin(theta)) * radius) + center));
  }

  // Setting rest distances for springs
  for (int rdIdx = 0; rdIdx < num_points; rdIdx++) {
    body.rest_distance.push_back(cf_len(body.points[(rdIdx + 1) % num_points].position - body.points[rdIdx].position));
  }

  // have the option here to add internal springs for more stabliliy within the structure
  // not sure if I like the behavior of the wheels with the extra interal springs
  for (int sIdx = 0; sIdx < num_points; sIdx++) {
    int nextIdx = (sIdx + 4) % num_points;
    body.springs.emplace_back(sIdx, nextIdx, cf_len(body.points[nextIdx].position - body.points[sIdx].position), 25.f);
  }

  // rest of data 
  body.gas_force = gas_force;
  body.num_points = num_points;
  body.spring_force = spring_force;
  body.damping_factor = 300.f;
  body.previous_angle = 0.f;

  body.max_omega = 5.0f;
  // used for when we need to divide by amount of points
  // makes it a little faster and less intense
  body.inv_np = 1.f / (float)num_points;

  return body;

}

void check_border_collisions(Point *p, float dt, float elasticity) {
  if (p->position.y < -(SCREEN_HEIGHT / 2)) {
    p->position.y = -(SCREEN_HEIGHT / 2.f);

    v2 vn = cf_v2(0, -1) * dot(cf_v2(0, -1), p->velocity);
    auto vt = p->velocity - vn;

    vn *= - elasticity;
    float friction = 100;
    vt *= std::exp(- friction * dt);

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
  int n = body->num_points;

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

  int n = body->points.size();

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
  v2 body2_com = calc_soft_body_center_of_mass(body2->points.data(), num_points2);

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
      //gameState.debug_drawCollisionPoint = true;
      //gameState.paused = true;
      handleCollision(point, closest_point,  0.5f);
    }
  }
}

float calc_soft_body_rotation_angle(SoftBody *body, v2 com) {
  // F = -kx * dt
  // add F to velocity
  float a = 0.f;
  float b = 0.f;
  for (int i = 0; i < body->num_points; i++) {
    v2 r = body->points[i].position - com;
    a += dot(r, body->anchor_vertex[i]);
    b += cross(r, body->anchor_vertex[i]);
  }
  float angle = -atan2(b, a);
  return angle;
}

v2 rotate(v2 point, float angle) {
  SinCos rotation = sincos(angle);
  v2 rotatedPoint = mul(rotation, point);
  return rotatedPoint;
}

v2 calc_soft_body_average_velocity(Point *points, int length) {
  v2 vel = {};
  for (int i = 0; i < length; i++) {
    vel += points[i].velocity;
  }
  vel = vel / float(length);
  return vel;
}

v2 calculateWheelAxel(PressureBody wheel) {
  v2 wheel_axl = cf_v2(0, 0);
  int cnt = 0;

  for (int i = 0; i < wheel.num_points; i += 2) {
    wheel_axl += wheel.points[i].position;

    cnt++;
  }

  wheel_axl /= (float)cnt;

  return wheel_axl;
}

v2 calculateWeightedAverage(SoftBody *body, int *idxs, float *weights, int length) {
  v2 avg = cf_v2(0, 0);
  float weights_sum = 0;
  for (int i = 0; i < length; i++) {
    float weight = weights[i];
    int idx = idxs[i];
    avg+= body->points[idx].position * weight;
    weights_sum += weight;
  }
  return avg / weights_sum;
}

void checkMouseDown(float dt) {
  if (cf_mouse_down(MOUSE_BUTTON_LEFT)) {

    float mouse_x = cf_mouse_x() - (SCREEN_WIDTH / 2);
    float mouse_y = (cf_mouse_y() - (SCREEN_HEIGHT / 2)) * -1;
    v2 mouse_center = V2(mouse_x, mouse_y);

    v2 distance = mouse_center - gameState.last_mousedown;
    v2 velocity = distance / dt;

    for (int i = 0; i < gameState.num_bodies; i++) {

      SoftBody *body = gameState.bodies[i];

      // find com for body
      v2 com = calc_soft_body_center_of_mass(body->points.data(), body->num_points);

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
      int n = body->points.size(); 

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
        for (int i = 0; i < body->num_points; i++) {
          body->points[i].position = mouse_center + body->anchor_vertex[i];
        }
      } else {
        body->clicked = false;
      }

      gameState.last_mousedown = mouse_center;
    }
  }
}

void set_car_torque(float t) {
  Car *car = &gameState.car;
  car->wheels[0]->torque = t;
  car->wheels[1]->torque = t;
}

float absf(float x) {
  if (x > 0.f) { return x; }
  return x * -1.f;
}

void check_inputs(float dt) {
  if (cf_key_down(CF_KEY_A)) {
    set_car_torque(-1.f);
  } else if (cf_key_down(CF_KEY_D)) {
    set_car_torque(1.f); 
  } else {
    set_car_torque(0);
  }
}

float rand_float(float min, float max) {
    return min + (rand() / (float)RAND_MAX) * (max - min);
}

v2 randomOffset() {
  return cf_v2(rand_float(-300, 300), 0);
}

void addSoftBody() {
  SoftBody *newBody = new SoftBody(makeSoftBody(randomOffset()));

  gameState.bodies.push_back(newBody);
  gameState.num_bodies++;
}

void update_soft_body(SoftBody *body, float dt) {

  checkMouseDown(dt);

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

  v2 com = calc_soft_body_center_of_mass(body->points.data(), body->num_points);

  // collision
  for (int i = 0; i < gameState.num_bodies; i++) {
    SoftBody *body = gameState.bodies[i];

    for (int i = 0; i < body->num_points; i++) {
      Point *point = &body->points[i];
      check_border_collisions(point, dt, -0.5);
    }

    for (int j = 0; j < gameState.num_bodies; j++) {
      if (j == i) {
        // skip self collision
        continue;
      }
      SoftBody *body2 = gameState.bodies[j];

      for (int k = 0; k < body->num_points; k++) {
        Point *point = &body->points[k];
        checkBodyCollision(point, body2, body->num_points, body2->num_points);
      }
    }
  }

  // constraints
  float angle = calc_soft_body_rotation_angle(body, com);
  v2 avg_vel = calc_soft_body_average_velocity(body->points.data(), body->num_points);

  for (int i = 0; i < body->num_points; i++) {

    Point *point = &body->points[i];
    v2 targetVertex = com + rotate(body->anchor_vertex[i], angle);
    v2 x = targetVertex - point->position;
    v2 target_vertex_corrected = body->anchor_vertex[i] + com;

    // spring force
    v2 spring_force = x * gameState.k_springForce * dt;
    point->velocity += spring_force;
  }

 // Update springs
  for (int springIndex = 0; springIndex < body->springs.size(); springIndex++) {
    Spring s = body->springs[springIndex];
    Point* c = &body->points[s.indexA];
    Point* d = &body->points[s.indexB];

    v2 delta = d->position - c->position;
    float distance = cf_len(delta);
    v2 direction = cf_safe_norm(delta);

    v2 required_delta = delta * (s.rest_distance / distance);
    v2 force = (required_delta - delta) * s.spring_force;

    // spring force
    c->velocity -= force * dt;
    d->velocity += force * dt;

    float vrel = dot(d->velocity - c->velocity, direction);
    float damping_factor = exp(-gameState.spring_damping * dt);
    float new_vrel = vrel * damping_factor;
    float vrel_delta = new_vrel - vrel;

    // damping
    c->velocity -= direction * (vrel_delta / 2.0);
    d->velocity += direction * (vrel_delta / 2.0);
  }
  // bascially just damping between every edge treating it as a spring

  int n = body->num_points;

  for (int i = 0; i < body->num_points; i++) {

    Point *point = &body->points[i];
    // this gets the next point to calc rest distance
    int next_idx = (i + 1) % n;
    Point *next_point = &body->points[next_idx];
    v2 rest_distance_vec = body->anchor_vertex[next_idx] - body->anchor_vertex[i];
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

float get_volume(PressureBody body) {
	float volume = 0;

	int n = body.num_points;

	for (int i = 0; i < n; i++) {
		float x1 = body.points[i].position.x;
		float y1 = body.points[i].position.y;
		float x2 = body.points[(i + 1) % n].position.x;
		float y2 = body.points[(i + 1) % n].position.y;

		float r12d = sqrt(
      (x1 - x2) * (x1 - x2) +
      (y1 - y2) * (y1 - y2)
    );

    v2 norm = cf_safe_norm(body.points[(i + 1) % n].position - body.points[i].position);

    volume += 0.5f * fabs(x1 - x2) * fabs(norm.y) * r12d;
	}

	return volume;

}

v2 calculateNormal(v2 a, v2 b) {
  v2 delta = cf_safe_norm(b - a);
  return cf_v2(- delta.y, delta.x);
}

float clampf(float min, float max, float val) {
  if (val < min) { return min; } 
  if (val > max) { return max; } 
  
  return val;
}

inline v2 get_perpindicular(v2 a) {
  return cf_v2(-a.y, a.x);
}

inline bool is_ccw( const v2 A, const v2 B ){
  v2 perp = get_perpindicular(A);
  float dot = cf_dot(perp, B);
  return (dot >= 0.0f);
}

float derive_wheel_omega(PressureBody *wheel, float dt) {
  float angle = 0.f;
  int original_sign = 1;
  float original_angle = 0.f;
  for (int i = 0; i < wheel->points.size(); i++) {
    Point p = wheel->points[i];
    
    v2 prev_norm = cf_norm(p.prev_position);
    v2 cur_norm = cf_norm(p.position);

    // clamp cus accuracy
    float dot = clampf(-1.f, 1.f, cf_dot(prev_norm, cur_norm));
    
    float cur_angle = (float)acos(dot);
    // check if it counter clock wise to determine sign
    if (!is_ccw(prev_norm, cur_norm)) { cur_angle = - cur_angle; }

    if (i == 0) {
      original_sign = cur_angle >= 0.f ? 1 : -1;
      original_angle = cur_angle;
    } else {
      float diff = (cur_angle - original_angle);
      int cur_sign = cur_angle >= 0.0f ? 1 : -1;

      if ((absf(diff) > M_PI) && (cur_sign != original_sign)) {
        cur_angle = (cur_sign == -1) ? (M_PI + M_PI + cur_angle) : ((M_PI - cur_angle) - M_PI);
      }
    }

    angle += cur_angle;
  }

  angle *= wheel->inv_np;
  
  float delta_angle = angle - wheel->previous_angle;
  if (absf(delta_angle) >= M_PI) {

			if (delta_angle < 0.0f)
				delta_angle = delta_angle + TWO_PI;
			else
				delta_angle = delta_angle - TWO_PI;
		
  }

  float omega = delta_angle / dt;
  wheel->previous_angle = angle;

  return omega;
}

void update_pressure_body(PressureBody *body, float dt) {

  // integrate gravity
  body->gas_force = gameState.gas_force;

  for (int g = 0; g < body->num_points; g++) {
    Point *p = &body->points[g];
    p->velocity += gameState.gravity * dt;
    p->position += p->velocity * dt;
  }

  //check collision
  for (int point_idx = 0; point_idx < body->num_points; point_idx++) {
    check_border_collisions(&body->points[point_idx], dt, 0.5);
  }

  // satisfy contraints

  for (int rd_idx = 0; rd_idx < body->num_points; rd_idx++) {
    int next_idx = (rd_idx + 1) % body->num_points;
    float rd = body->rest_distance[rd_idx];

    Point *a = &body->points[rd_idx];
    Point *b = &body->points[next_idx];

    v2 delta = b->position - a->position;
    float distance = cf_len(delta);
    v2 direction = cf_safe_norm(delta);

    v2 required_delta = delta * (rd / distance);
    v2 force = (required_delta - delta) * body->spring_force;

    // spring force
    a->velocity -= force * dt;
    b->velocity += force * dt;

    float vrel = dot(b->velocity - a->velocity, direction);
    float damping_factor = exp(- gameState.spring_damping * dt);
    float new_vrel = vrel * damping_factor;
    float vrel_delta = new_vrel - vrel;

    // damping
    a->velocity -= direction * (vrel_delta / 2.0);
    b->velocity += direction * (vrel_delta / 2.0);
  }

  //apply gas force
  float volume = get_volume(*body);

  for (int j = 0; j < body->num_points; j++) {
    Point *p0 = &body->points[j];
    Point *p1 = &body->points[(j + 1) % body->num_points];

    float x0 = p0->position.x;
    float y0 = p0->position.y;
    float x1 = p1->position.x;
    float y1 = p1->position.y;

    float r12d = sqrt(
      (x0 - x1) * (x0 - x1) +
      (y0 - y1) * (y0 - y1)
    );

    float pressurev = r12d * body->gas_force * (1.0f / volume);
    v2 normal = calculateNormal(p1->position, p0->position);

    p0->velocity += normal * pressurev;
    p1->velocity += normal * pressurev;
  }

  // apply springs
  for (int springIndex = 0; springIndex < body->springs.size(); springIndex++) {
    Spring s = body->springs[springIndex];
    Point* c = &body->points[s.indexA];
    Point* d = &body->points[s.indexB];

    v2 delta = d->position - c->position;
    float distance = cf_len(delta);
    v2 direction = cf_safe_norm(delta);

    v2 required_delta = delta * (s.rest_distance / distance);
    v2 force = (required_delta - delta) * s.spring_force;

    // spring force
    c->velocity -= force * dt;
    d->velocity += force * dt;

    float vrel = dot(d->velocity - c->velocity, direction);
    float damping_factor = exp(-gameState.spring_damping * dt);
    float new_vrel = vrel * damping_factor;
    float vrel_delta = new_vrel - vrel;

    // damping
    c->velocity -= direction * (vrel_delta / 2.0);
    d->velocity += direction * (vrel_delta / 2.0);
  }

  // This shit broken as hell
  // Okay maybe not broken as hell. Just need to figure out why the ratio isn't hitting 0
  // Two above messages were a day apart and I can indeed confirm it's broken as hell
  // ts returning a ratio of over 100% LMAOOOOOOOOO
  // ^^ probably just a byprodct of unrealistic forces. would happen if the following was true:
  // -- pos - prev_pos > FOUR_PI 
  // -- since
  // -- pos - prev_pos - TWO_PI > TWO_PI
  // oh yeah, fiziks :D
  float derived_omega = derive_wheel_omega(body, dt);
  std::cout << absf(((derived_omega > 0) ? body->max_omega : -body->max_omega) - derived_omega) / body->max_omega << std::endl;
	std::cout << derived_omega << std::endl;
  float omega_factor = (absf(((derived_omega > 0) ? body->max_omega : -body->max_omega) - derived_omega) / body->max_omega) * body->torque;

  // TODO figure this the fuck out
  v2 axel = calculateWheelAxel(*body);
  for (int i = 0; i < body->num_points; i++) { 
    Point *p = &body->points[i];

    v2 toPt = p->position - axel; 
    toPt = rotate(toPt, NEG_PI_OVER_ONE_POINT_TWO);
    
    p->velocity += toPt * omega_factor; 
  }  
}

void update_car(Car *car, float dt) {

  SoftBody *car_body = car->car_body;
  PressureBody *back_wheel = car->wheels[0];
  PressureBody *front_wheel = car->wheels[1];

  // weights and points for avg
  int back_axl_idx[4] = {10, 11, 0, 1};
  int front_axl_idx[4] = {9, 8, 7, 5};
  float weights[4] = {3, 2, 1, 1};
  float weights_sum = 7;

  // axls
  v2 car_back_axl = calculateWeightedAverage(car_body, back_axl_idx, weights, 4);
  v2 car_front_axl = calculateWeightedAverage(car_body, front_axl_idx, weights, 4);
  v2 back_wheel_axl = calculateWheelAxel(*back_wheel);
  v2 front_wheel_axl = calculateWheelAxel(*front_wheel);

  // debug
  gameState.axls[0] = car_back_axl;
  gameState.axls[1] = car_front_axl;
  gameState.axls[2] = back_wheel_axl;
  gameState.axls[3] = front_wheel_axl;

  // back axel
  v2 back_delta = car_back_axl - back_wheel_axl;
  for (int i = 0; i < 4; i++) {
    Point *p1 = &car_body->points[back_axl_idx[i]];
    p1->prev_position = p1->position;
    p1->position -= back_delta / 2.0f;
    p1->velocity += (p1->position - p1->prev_position) / dt;
  }
  for (int i = 0; i < back_wheel->num_points; i++) {
    Point *p0 = &back_wheel->points[i];
    p0->prev_position = p0->position;
    p0->position += back_delta / 2.0f;
    p0->velocity += (p0->position - p0->prev_position) / dt;
  }
  // front axel
  v2 front_delta = car_front_axl - front_wheel_axl;
  for (int i = 0; i < 4; i++) {
    Point *p1 = &car_body->points[front_axl_idx[i]];
    p1->prev_position = p1->position;
    p1->position -= front_delta / 2.0f;
    p1->velocity += (p1->position - p1->prev_position) / dt;
  }
  for (int i = 0; i < front_wheel->num_points; i++) {
    Point *p0 = &front_wheel->points[i];
    p0->prev_position = p0->position;
    p0->position += front_delta / 2.0f;
    p0->velocity += (p0->position - p0->prev_position) / dt;
  }

  // TODO make this cleaner
  for (int i = 0; i < front_wheel->num_points; i++) {
    Point *p0 = &front_wheel->points[i];
    Point *p1 = &back_wheel->points[i];
    p0->prev_position = p0->position; 
    p1->prev_position = p1->position; 
  }
}

void calc_energy() {

  SoftBody *car_body = gameState.bodies[0];

  float energy = 0;
  for (int i = 0; i < car_body->num_points; i++) {
    Point *p = &car_body->points[i];
    energy += cf_len(p->velocity) * 0.5f;
  }

  energy /= car_body->num_points;
  gameState.debug_energyCar = energy;

  PressureBody *back_wheel = gameState.p_bodies[0];
  PressureBody *front_wheel = gameState.p_bodies[1];

  float energy_back = 0;
  float energy_front = 0;

  for (int i = 0; i < back_wheel->num_points; i++) {
    energy_back = cf_len(back_wheel->points[i].velocity) * 0.5f;
    energy_front = cf_len(front_wheel->points[i].velocity) * 0.5f;
  }

  energy_back /= back_wheel->num_points;
  energy_front /= front_wheel->num_points;

  gameState.debug_energyBackWheel = energy_back;
  gameState.debug_energyFrontWheel = energy_front;

}

void update(float dt) {
  
  //deriveWheelAngle(dt);

  check_inputs(dt);

  for (int i = 0; i < gameState.bodies.size(); i++) {
    SoftBody *body = gameState.bodies[i];
    update_soft_body(body, dt);
  }

  for (int i = 0; i < gameState.p_bodies.size(); i++) {
    PressureBody *body = gameState.p_bodies[i];
    update_pressure_body(body, dt);
  }

  update_car(&gameState.car, dt);
  //
  calc_energy();
}

void draw_soft_body(SoftBody *body) {

  int n = body->num_points;

  for (int i = 0; i < body->num_points; i++) {
    v2 a = body->points[i].position;
    v2 b = body->points[(i + 1) % n].position;

    cf_draw_line(a, b, 0.5f);
  }

  draw_push_color(cf_color_yellow());
  for (int i = 0; i < body->springs.size(); i++) {
    Spring s = body->springs[i];
    v2 a = body->points[s.indexA].position;
    v2 b = body->points[s.indexB].position;
    cf_draw_line(a, b, 0.3f);
  }
  draw_pop_color();

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

  v2 com = calc_soft_body_center_of_mass(body->points.data(), body->num_points);
  float angle = calc_soft_body_rotation_angle(body, com);

  if (gameState.debug_drawTargetShape) {
    draw_push_color(cf_color_green());
    for (int i = 0; i < body->num_points; i++) {
      SinCos rotation = sincos(angle);
      v2 rotatedAnchor = mul(rotation, body->anchor_vertex[i]);
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

  draw_pop_color();

  draw_push_color(cf_color_orange());
  for (int i = 0; i < 4; i++) {
    cf_draw_circle2(gameState.axls[i], RADIUS, 1.f);
  }
  draw_pop_color();
}

void drawPressureBody(PressureBody *body) {

  // draw lines
  int n = body->num_points;

  for (int i = 0; i < body->num_points; i++) {
    int nextIdx = (i + 1) % n;

    cf_draw_line(body->points[i].position, body->points[nextIdx].position , 0.5f);
  }

  // draw points
  draw_push_color(cf_color_red());
  for (int i = 0; i < body->num_points; i++) {
    cf_draw_circle2(body->points[i].position, RADIUS, 1.0f);
  }
  draw_pop_color();

  draw_push_color(cf_color_yellow());
  for (int i = 0; i < body->springs.size(); i++) {
    Spring s = body->springs[i];
    v2 a = body->points[s.indexA].position;
    v2 b = body->points[s.indexB].position;
    cf_draw_line(a, b, 0.3f);
  }
  draw_pop_color();
  
}

void main_loop(void *udata) {
  update(CF_DELTA_TIME_FIXED);
}

void init_scene() {
  gameState.num_bodies = 1;
  gameState.bodies.push_back(new SoftBody(MakeCarBody(cf_v2(0, -150))));
  gameState.num_gas_bodies = 2;
  gameState.p_bodies.push_back(new PressureBody(MakePressureBody(cf_v2(-20, -150), 300.0, 12)));
  gameState.p_bodies.push_back(new PressureBody(MakePressureBody(cf_v2(20, -150), 300.0, 12)));
  gameState.k_springForce = 100.f;
  gameState.spring_damping = 10.f;
  gameState.gas_force = 300.f;
  gameState.gravity = V2(0, -9.8f);
  gameState.debug_drawTargetShape = true;
  gameState.debug_drawCenterOfMass = true;
  gameState.last_mousedown = V2(0.0, 0.0);

  gameState.car = {};
  gameState.car.car_body = gameState.bodies[0];
  gameState.car.wheels[0] = gameState.p_bodies[0];
  gameState.car.wheels[1] = gameState.p_bodies[1];

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

  init_scene();

  CF_ASSERT(gameState.k_springForce * CF_DELTA_TIME_FIXED *
                CF_DELTA_TIME_FIXED <
            1.f);
  while (app_is_running()) {
    app_update(&main_loop);
    // All your game logic and updates go here...
    for (int i = 0; i < gameState.num_bodies; i++) {
      SoftBody *body = gameState.bodies[i];
      draw_soft_body(body);
    }

    for (int i = 0; i < gameState.num_gas_bodies; i++) {
      PressureBody *body = gameState.p_bodies[i];
      drawPressureBody(body);
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

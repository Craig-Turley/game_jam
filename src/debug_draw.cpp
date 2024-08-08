#include "debug_draw.h"

#include "main.h"
#include <cstdio>
#include <imgui/imgui.h>

void drawImgui(GameState *state)
{
    ImGui::Begin("Settings");
    ImGui::DragFloat("spring force", &state->k_springForce, 1.0f, 0.f, 300.f);
    ImGui::DragFloat("spring damping", &state->spring_damping, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat2("gravity", (float*)&state->gravity, 1.0f, -500.0f, 500.0f);
    ImGui::Checkbox("draw target shape", &state->debug_drawTargetShape);
    ImGui::Checkbox("draw center of mass", &state->debug_drawCenterOfMass);
    for(int i = 0; i < 4; i++){
      char title[32];
      std::sprintf(title, "Position %d", i);
      ImGui::LabelText(title, "%.3f %.3f", state->body1.points[i].last_position.x, state->body1.points[i].last_position.y);
    }
    for(int i = 0; i < 4; i++){
      char title[32];
      std::sprintf(title, "Velocity %d", i);
      ImGui::LabelText(title, "%.3f %.3f", state->body1.points[i].last_velocity.x, state->body1.points[i].last_velocity.y);
    }
    for(int i = 0; i < 4; i++){
      char title[32];
      std::sprintf(title, "Damping %d", i);
      ImGui::LabelText(title, "%.3f", state->body1.points[i].last_damping);
    }
    for(int i = 0; i < 4; i++){
      char title[32];
      std::sprintf(title, "Anchor Dist %d", i);
      ImGui::LabelText(title, "%.3f %.3f", state->body1.points[i].last_anchor_dist.x, state->body1.points[i].last_anchor_dist.y);
    }
    ImGui::End();
}

#include "debug_draw.h"

#include "main.h"
#include <cstdio>
#include <imgui/imgui.h>
#include <iostream>

void drawImgui(GameState *state)
{
    ImGui::Begin("Settings");
    ImGui::DragFloat("spring force", &state->k_springForce, 1.0f, 0.f, 300.f);
    ImGui::DragFloat("spring damping", &state->spring_damping, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat2("gravity", (float*)&state->gravity, 1.0f, -500.0f, 500.0f);
    ImGui::DragFloat("gas force", &state->gas_force, 1.0f, 0.0f, 1000.f);
    ImGui::Checkbox("draw target shape", &state->debug_drawTargetShape);
    ImGui::Checkbox("draw center of mass", &state->debug_drawCenterOfMass);
    ImGui::Checkbox("draw bounding box", &state->debug_drawBoundingBox);
    if(ImGui::Button("add body", ImVec2(80,20))) {
      addSoftBody();       
    };
    if(ImGui::Button("game over", ImVec2(80,20))) {
      state->game_over = true;
    };
		char area[5];
		char energy_body[13];
		std::sprintf(energy_body, "energy_body");
		ImGui::LabelText(energy_body, "%.3f", state->debug_energyCar);
		char energy_back[13];
		std::sprintf(energy_back, "energy_back");
		ImGui::LabelText(energy_back, "%.3f", state->debug_energyBackWheel);
		char energy_front[13];
		std::sprintf(energy_front, "energy_front");
		ImGui::LabelText(energy_front, "%.3f", state->debug_energyFrontWheel);
    ImGui::End();
}

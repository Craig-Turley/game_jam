#include "debug_draw.h"

#include "main.h"
#include <imgui/imgui.h>

void drawImgui(GameState *state)
{
    ImGui::Begin("Settings");
    ImGui::DragFloat("spring force", &state->k_springForce, 1.0f, 0.f, 300.f);
    ImGui::DragFloat("spring damping", &state->spring_damping, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat2("gravity", (float*)&state->gravity, 1.0f, -500.0f, 500.0f);
    ImGui::Checkbox("draw target shape", &state->debug_drawTargetShape);
    ImGui::Checkbox("draw center of mass", &state->debug_drawCenterOfMass);
    ImGui::End();
}

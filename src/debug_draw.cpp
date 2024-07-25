#include "main.h"
#include <imgui/imgui.h>

static void drawImgui(GameState *state)
{
    ImGui::Begin("Settings");
    ImGui::DragFloat("spring force", &state->k_springForce);
    ImGui::DragFloat("spring damping", &state->spring_damping, 1.0f, 0.0f, 1.0f);
    ImGui::Checkbox("draw target shape", &state->debug_drawTargetShape);
    ImGui::Checkbox("draw center of mass", &state->debug_drawCenterOfMass);
    ImGui::End();
}

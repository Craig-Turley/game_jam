#include "main.h"
#include <imgui/imgui.h>

static void drawImgui(GameState *state)
{
    ImGui::Begin("Settings");
    ImGui::DragFloat("spring force", &state->k_springForce);
    ImGui::End();
}
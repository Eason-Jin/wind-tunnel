#include "app/ui/Theme.h"

#include <cstdarg>
#include <filesystem>
#include <string>

namespace app::ui {

void applyTheme()
{
    ImGuiIO& io = ImGui::GetIO();
    const std::string font = std::string(WT_PROJECT_DIR) + "/assets/fonts/Roboto-Medium.ttf";
    if (std::filesystem::exists(font))
        io.Fonts->AddFontFromFileTTF(font.c_str(), kBaseFontSize);

    ImGuiStyle& s = ImGui::GetStyle();
    s.FontSizeBase = kBaseFontSize;
    s.WindowPadding = ImVec2(14, 12);
    s.FramePadding = ImVec2(9, 6);
    s.ItemSpacing = ImVec2(8, 8);
    s.ItemInnerSpacing = ImVec2(6, 5);
    s.IndentSpacing = 14;
    s.ScrollbarSize = 10;
    s.GrabMinSize = 10;
    s.WindowRounding = 0;
    s.ChildRounding = 6;
    s.FrameRounding = 6;
    s.PopupRounding = 6;
    s.GrabRounding = 6;
    s.ScrollbarRounding = 6;
    s.TabRounding = 6;
    s.WindowBorderSize = 0;
    s.FrameBorderSize = 0;
    s.PopupBorderSize = 1;
    s.SeparatorTextBorderSize = 1;

    ImVec4* c = s.Colors;
    const ImVec4 bg(0.105f, 0.118f, 0.137f, 1.0f);      // panels
    const ImVec4 bgDeep(0.078f, 0.086f, 0.102f, 1.0f);  // toolbar / status bar
    const ImVec4 frame(0.157f, 0.176f, 0.204f, 1.0f);   // inputs
    const ImVec4 frameHi(0.200f, 0.224f, 0.259f, 1.0f); // hover
    const ImVec4 accent(0.239f, 0.545f, 0.992f, 1.0f);
    const ImVec4 accentHi(0.369f, 0.627f, 1.0f, 1.0f);
    const ImVec4 text(0.886f, 0.910f, 0.941f, 1.0f);
    const ImVec4 muted(0.545f, 0.580f, 0.627f, 1.0f);

    c[ImGuiCol_Text] = text;
    c[ImGuiCol_TextDisabled] = muted;
    c[ImGuiCol_WindowBg] = bg;
    c[ImGuiCol_ChildBg] = ImVec4(0.125f, 0.141f, 0.165f, 1.0f);
    c[ImGuiCol_PopupBg] = ImVec4(0.125f, 0.141f, 0.165f, 0.98f);
    c[ImGuiCol_MenuBarBg] = bgDeep;
    c[ImGuiCol_Border] = ImVec4(1, 1, 1, 0.08f);
    c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_FrameBg] = frame;
    c[ImGuiCol_FrameBgHovered] = frameHi;
    c[ImGuiCol_FrameBgActive] = frameHi;
    c[ImGuiCol_TitleBg] = bgDeep;
    c[ImGuiCol_TitleBgActive] = bgDeep;
    c[ImGuiCol_TitleBgCollapsed] = bgDeep;
    c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ScrollbarGrab] = frameHi;
    c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.3f, 0.33f, 0.38f, 1);
    c[ImGuiCol_ScrollbarGrabActive] = accent;
    c[ImGuiCol_CheckMark] = accentHi;
    c[ImGuiCol_SliderGrab] = accent;
    c[ImGuiCol_SliderGrabActive] = accentHi;
    c[ImGuiCol_Button] = frame;
    c[ImGuiCol_ButtonHovered] = frameHi;
    c[ImGuiCol_ButtonActive] = accent;
    c[ImGuiCol_Header] = ImVec4(0.239f, 0.545f, 0.992f, 0.22f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.239f, 0.545f, 0.992f, 0.32f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.239f, 0.545f, 0.992f, 0.45f);
    c[ImGuiCol_Separator] = ImVec4(1, 1, 1, 0.07f);
    c[ImGuiCol_SeparatorHovered] = accent;
    c[ImGuiCol_SeparatorActive] = accentHi;
    c[ImGuiCol_ResizeGrip] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_Tab] = frame;
    c[ImGuiCol_TabHovered] = frameHi;
    c[ImGuiCol_TabSelected] = accent;
    c[ImGuiCol_PlotHistogram] = accent;
    c[ImGuiCol_TextSelectedBg] = ImVec4(0.239f, 0.545f, 0.992f, 0.35f);
    c[ImGuiCol_NavCursor] = accentHi;
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0, 0, 0, 0.55f);
}

void sectionHeader(const char* label)
{
    ImGui::Dummy(ImVec2(0, 2));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(colour::kTextMuted));
    ImGui::PushFont(nullptr, kBaseFontSize * 0.85f);
    ImGui::TextUnformatted(label);
    ImGui::PopFont();
    ImGui::PopStyleColor();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x, p.y - 3), ImVec2(p.x + w, p.y - 3), IM_COL32(255, 255, 255, 18));
    ImGui::Dummy(ImVec2(0, 2));
}

bool bigButton(const char* label, ImU32 colour, ImU32 hover, ImVec2 size)
{
    ImGui::PushStyleColor(ImGuiCol_Button, colour);
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hover);
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, colour);
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255, 255, 255));
    const bool clicked = ImGui::Button(label, size);
    ImGui::PopStyleColor(4);
    return clicked;
}

bool segmented(const char* id, int* current, const char* const labels[], int count, float width)
{
    ImGui::PushID(id);
    bool changed = false;
    const float spacing = 2.0f;
    const float w = (width - spacing * static_cast<float>(count - 1)) / static_cast<float>(count);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(spacing, 0));
    for (int i = 0; i < count; ++i) {
        if (i > 0)
            ImGui::SameLine();
        const bool selected = *current == i;
        if (selected) {
            ImGui::PushStyleColor(ImGuiCol_Button, colour::kAccent);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, colour::kAccentHover);
            ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 255, 255, 255));
        }
        if (ImGui::Button(labels[i], ImVec2(w, 0)) && !selected) {
            *current = i;
            changed = true;
        }
        if (selected)
            ImGui::PopStyleColor(3);
    }
    ImGui::PopStyleVar();
    ImGui::PopID();
    return changed;
}

void hint(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(colour::kTextMuted));
    ImGui::TextWrappedV(fmt, args);
    ImGui::PopStyleColor();
    va_end(args);
}

} // namespace app::ui

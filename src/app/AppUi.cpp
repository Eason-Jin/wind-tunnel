// The slicer-style interface: toolbar, model/simulation panel on the left,
// view layers on the right, status bar at the bottom, 3D scene in between.

#include "app/App.h"

#include "app/ui/Theme.h"
#include "solvers/openfoam/OpenFoamSolver.h"
#include "solvers/synthetic/SyntheticSolver.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace app {

namespace {

constexpr ImGuiWindowFlags kPanelFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;

struct QualityPreset {
    const char* name;
    int gridCellsX;
    int refinementLevel;
    int iterations;
    const char* estimate; // car-sized body, 8 cores
};
constexpr QualityPreset kQuality[] = {
    {"Draft", 96, 2, 250, "about 20 s"},
    {"Normal", 128, 3, 400, "about 40 s"},
    {"Fine", 176, 4, 600, "about 5 min"},
};

struct UnitPreset {
    const char* name;
    float toMetres;
};
constexpr UnitPreset kUnits[] = {{"m", 1.0f}, {"cm", 0.01f}, {"mm", 0.001f}, {"in", 0.0254f}};

// Friendly names for the layers list, keyed by RenderPass::name().
struct LayerInfo {
    const char* pass;
    const char* label;
    const char* help;
};
constexpr LayerInfo kLayers[] = {
    {"Model", "Body surface", "Shaded model, coloured by surface pressure (Cp)"},
    {"Streamline", "Streamlines", "Paths of air released from a rake upstream"},
    {"Particle", "Smoke", "Animated tracer particles (press Play)"},
    {"Vortices", "Vortex cores", "Swirling structures (Q-criterion)"},
    {"Slice", "Section plane", "Cut through the flow, with in-plane streamlines"},
    {"Tunnel", "Tunnel outline", "Test-section box, floor grid and flow arrow"},
};

const LayerInfo* layerInfo(const char* passName)
{
    for (const auto& l : kLayers)
        if (std::strcmp(l.pass, passName) == 0)
            return &l;
    return nullptr;
}

float speedToDisplay(float ms, int unit) { return unit == 0 ? ms * 3.6f : ms; }
float speedFromDisplay(float v, int unit) { return unit == 0 ? v / 3.6f : v; }
const char* speedUnitName(int unit) { return unit == 0 ? "km/h" : "m/s"; }

// Play/pause/cancel button with a drawn icon (the bundled font has no symbols).
bool playButton(const char* label, int icon, ImU32 colour, ImU32 hover, ImVec2 size)
{
    const ImVec2 pos = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(label, size);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), hovered ? hover : colour, size.y * 0.5f);

    const float iconSize = size.y * 0.36f;
    const ImVec2 c(pos.x + size.y * 0.62f, pos.y + size.y * 0.5f);
    const ImU32 white = IM_COL32(255, 255, 255, 255);
    if (icon == 0) { // play triangle
        dl->AddTriangleFilled(ImVec2(c.x - iconSize * 0.45f, c.y - iconSize * 0.6f),
                              ImVec2(c.x - iconSize * 0.45f, c.y + iconSize * 0.6f), ImVec2(c.x + iconSize * 0.6f, c.y), white);
    } else if (icon == 1) { // pause bars
        const float w = iconSize * 0.32f, h = iconSize * 0.6f;
        dl->AddRectFilled(ImVec2(c.x - w * 1.6f, c.y - h), ImVec2(c.x - w * 0.4f, c.y + h), white, 1.5f);
        dl->AddRectFilled(ImVec2(c.x + w * 0.4f, c.y - h), ImVec2(c.x + w * 1.6f, c.y + h), white, 1.5f);
    } else { // stop square
        const float h = iconSize * 0.5f;
        dl->AddRectFilled(ImVec2(c.x - h, c.y - h), ImVec2(c.x + h, c.y + h), white, 2.0f);
    }
    const ImVec2 ts = ImGui::CalcTextSize(label);
    const float textX = pos.x + size.y * 0.62f + iconSize + 8.0f;
    dl->AddText(ImVec2(textX, pos.y + (size.y - ts.y) * 0.5f), white, label);
    return clicked;
}

// Vertically centre the next widget inside a bar of the given height.
void centreInBar(float barHeight, float widgetHeight)
{
    ImGui::SetCursorPosY((barHeight - widgetHeight) * 0.5f);
}

} // namespace

render::RenderPass* App::findPass(const char* name) const
{
    for (const auto& p : passes_)
        if (std::strcmp(p->name(), name) == 0)
            return p.get();
    return nullptr;
}

void App::applyQualityPreset()
{
    if (quality_ < 0 || quality_ > 2)
        return;
    params_.gridCellsX = kQuality[quality_].gridCellsX;
    params_.refinementLevel = kQuality[quality_].refinementLevel;
    params_.iterations = kQuality[quality_].iterations;
}

bool App::needsSolve() const
{
    if (field_.empty())
        return true;
    if (std::abs(fieldSpeed_ - params_.inletSpeed) > 1e-3f)
        return true;
    return solverKind_ == SolverKind::OpenFoam && fieldIsPreview_;
}

void App::setPlaying(bool playing)
{
    playing_ = playing;
    if (playing && !layersShownOnPlay_) {
        // First Play: bring in the animated / aero layers so there is something to watch.
        for (const char* name : {"Particle", "Vortices"})
            if (render::RenderPass* p = findPass(name))
                p->enabled = true;
        layersShownOnPlay_ = true;
    }
    for (auto& p : passes_)
        p->setPlaying(playing);
}

void App::onPlayPressed()
{
    if (running_) {
        cancel_ = true;
        playAfterSolve_ = false;
        status_ = "Cancelling...";
        return;
    }
    if (!needsSolve()) {
        setPlaying(!playing_);
        return;
    }
    if (solverKind_ == SolverKind::Synthetic) {
        previewSyntheticField();
        status_ = "Instant preview (potential flow) - switch to OpenFOAM for real CFD";
        setPlaying(true);
        return;
    }
    playAfterSolve_ = true;
    setPlaying(false);
    startSolver(SolverKind::OpenFoam);
}

void App::updateSceneRect(float displayW, float displayH, float fbScale)
{
    const float left = ui::kLeftPanelWidth, right = ui::kRightPanelWidth;
    const float top = ui::kToolbarHeight, bottom = ui::kStatusHeight;
    const float w = std::max(displayW - left - right, 1.0f);
    const float h = std::max(displayH - top - bottom, 1.0f);
    sceneRect_.x = static_cast<int>(left * fbScale);
    sceneRect_.y = static_cast<int>(bottom * fbScale); // GL origin is bottom-left
    sceneRect_.w = static_cast<int>(w * fbScale);
    sceneRect_.h = static_cast<int>(h * fbScale);
}

void App::drawUi()
{
    const ImGuiIO& io = ImGui::GetIO();
    updateSceneRect(io.DisplaySize.x, io.DisplaySize.y, io.DisplayFramebufferScale.x);
    drawToolbar();
    drawLeftPanel();
    drawRightPanel();
    drawStatusBar();
}

void App::drawToolbar()
{
    const ImGuiIO& io = ImGui::GetIO();
    const float H = ui::kToolbarHeight;
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, H));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(20, 22, 26, 255));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14, 0));
    ImGui::Begin("##toolbar", nullptr, kPanelFlags);

    const float frameH = ImGui::GetFrameHeight();

    // Brand.
    ImGui::PushFont(nullptr, ui::kBaseFontSize * 1.35f);
    const float titleH = ImGui::GetTextLineHeight();
    centreInBar(H, titleH);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kAccentHover), "WIND");
    ImGui::SameLine(0, 4);
    centreInBar(H, titleH);
    ImGui::TextUnformatted("TUNNEL");
    ImGui::PopFont();

    // File actions.
    ImGui::SameLine(0, 28);
    centreInBar(H, frameH);
    ImGui::BeginDisabled(running_);
    if (ImGui::Button("  Open STL...  "))
        ImGui::OpenPopup("##open");
    ImGui::SameLine();
    centreInBar(H, frameH);
    if (ImGui::Button("  Samples  "))
        ImGui::OpenPopup("##samples");
    ImGui::EndDisabled();

    // Temporary open popup: a path box (replaced by the native file dialog).
    if (ImGui::BeginPopup("##open")) {
        static char path[1024] = "";
        ImGui::SetNextItemWidth(420);
        ImGui::InputTextWithHint("##path", "/path/to/model.stl", path, sizeof path);
        ImGui::SameLine();
        if (ImGui::Button("Open")) {
            if (loadBody(path, bodyScale_)) {
                previewSyntheticField();
                frameCamera();
                setPlaying(false);
            }
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    if (ImGui::BeginPopup("##samples")) {
        if (ImGui::Selectable("Test sphere")) {
            if (loadBody("", 1.0f)) {
                previewSyntheticField();
                frameCamera();
                setPlaying(false);
            }
        }
        ImGui::EndPopup();
    }

    // Centre group: speed + play.
    const float speedW = 110.0f, unitW = 80.0f, playW = 150.0f, playH = 40.0f;
    const float groupW = ImGui::CalcTextSize("Wind speed").x + 10 + speedW + 4 + unitW + 18 + playW;
    ImGui::SameLine();
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX() + 20.0f, (io.DisplaySize.x - groupW) * 0.5f));
    centreInBar(H, frameH);
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kTextMuted), "Wind speed");
    ImGui::SameLine(0, 10);
    centreInBar(H, frameH);
    int unit = static_cast<int>(speedUnit_);
    float shown = speedToDisplay(params_.inletSpeed, unit);
    ImGui::SetNextItemWidth(speedW);
    ImGui::BeginDisabled(running_);
    if (ImGui::DragFloat("##speed", &shown, unit == 0 ? 0.5f : 0.1f, unit == 0 ? 1.0f : 0.3f, unit == 0 ? 600.0f : 170.0f,
                         unit == 0 ? "%.0f" : "%.1f"))
        params_.inletSpeed = std::max(speedFromDisplay(shown, unit), 0.1f);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        if (fieldIsPreview_)
            previewSyntheticField();
        else if (needsSolve())
            status_ = "Wind speed changed - press Play to re-run the simulation";
    }
    ImGui::EndDisabled();
    ImGui::SameLine(0, 4);
    centreInBar(H, frameH);
    ImGui::SetNextItemWidth(unitW);
    if (ImGui::Combo("##unit", &unit, "km/h\0m/s\0"))
        speedUnit_ = static_cast<SpeedUnit>(unit);

    ImGui::SameLine(0, 18);
    centreInBar(H, playH);
    if (running_) {
        if (playButton("Cancel", 2, ui::colour::kStop, ui::colour::kStopHover, ImVec2(playW, playH)))
            onPlayPressed();
    } else if (playing_) {
        if (playButton("Pause", 1, ui::colour::kAccent, ui::colour::kAccentHover, ImVec2(playW, playH)))
            onPlayPressed();
    } else {
        const bool solve = needsSolve() && solverKind_ == SolverKind::OpenFoam;
        if (playButton(solve ? "Simulate" : "Play", 0, ui::colour::kPlay, ui::colour::kPlayHover, ImVec2(playW, playH)))
            onPlayPressed();
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip(solve ? "Run the CFD simulation, then animate the flow (Space)" : "Animate the flow (Space)");
    }

    // Right: solver choice.
    const float solverW = 190.0f;
    ImGui::SameLine();
    ImGui::SetCursorPosX(io.DisplaySize.x - solverW - 14.0f);
    centreInBar(H, frameH);
    ImGui::SetNextItemWidth(solverW);
    int kind = static_cast<int>(solverKind_);
    ImGui::BeginDisabled(running_);
    if (ImGui::Combo("##solver", &kind, "Instant preview\0OpenFOAM CFD\0")) {
        if (kind == 1 && !solvers::OpenFoamSolver::available())
            status_ = "OpenFOAM not found (expected /usr/lib/openfoam/openfoam2406)";
        else
            solverKind_ = static_cast<SolverKind>(kind);
    }
    ImGui::EndDisabled();

    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

void App::drawLeftPanel()
{
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, ui::kToolbarHeight));
    ImGui::SetNextWindowSize(ImVec2(ui::kLeftPanelWidth, io.DisplaySize.y - ui::kToolbarHeight - ui::kStatusHeight));
    ImGui::Begin("##left", nullptr, kPanelFlags);
    const float fullW = ImGui::GetContentRegionAvail().x;

    // ---- Model ----------------------------------------------------------
    ui::sectionHeader("MODEL");
    {
        const std::string name = bodyPath_.empty() ? std::string("Test sphere")
                                                   : std::filesystem::path(bodyPath_).filename().string();
        ImGui::PushFont(nullptr, ui::kBaseFontSize * 1.15f);
        ImGui::TextUnformatted(name.c_str());
        ImGui::PopFont();
        const glm::vec3 s = body_.bounds().size();
        ui::hint("%.2f x %.2f x %.2f m  -  %zu triangles", s.x, s.y, s.z, body_.triangleCount());
    }

    ImGui::BeginDisabled(running_);
    ImGui::TextUnformatted("File units");
    const char* unitLabels[] = {kUnits[0].name, kUnits[1].name, kUnits[2].name, kUnits[3].name};
    if (ui::segmented("units", &unitsPreset_, unitLabels, 4, fullW) && !bodyPath_.empty()) {
        bodyScale_ = kUnits[unitsPreset_].toMetres;
        applyBodyTransform();
        previewSyntheticField();
        frameCamera();
        setPlaying(false);
    }

    ImGui::Dummy(ImVec2(0, 2));
    ImGui::TextUnformatted("Up axis in file");
    const char* upLabels[] = {"+Z", "+Y", "+X"};
    bool reorient = ui::segmented("up", &upAxis_, upLabels, 3, fullW);
    ImGui::Dummy(ImVec2(0, 2));
    ImGui::TextUnformatted("Rotate about up");
    const char* yawLabels[] = {"0", "90", "180", "270"};
    reorient |= ui::segmented("yaw", &yawSteps_, yawLabels, 4, fullW);
    if (reorient) {
        applyBodyTransform();
        previewSyntheticField();
        frameCamera();
        setPlaying(false);
    }
    ui::hint("Wind blows along +X. Point the nose into the wind (towards -X).");
    ImGui::EndDisabled();

    // ---- Simulation -----------------------------------------------------
    ImGui::Dummy(ImVec2(0, 6));
    ui::sectionHeader("SIMULATION");
    ImGui::BeginDisabled(running_);
    if (solverKind_ == SolverKind::OpenFoam) {
        ImGui::TextUnformatted("Quality");
        const char* qLabels[] = {kQuality[0].name, kQuality[1].name, kQuality[2].name};
        int q = std::min(quality_, 2);
        if (ui::segmented("quality", &q, qLabels, 3, fullW)) {
            quality_ = q;
            applyQualityPreset();
        }
        if (quality_ <= 2)
            ui::hint("Estimated run time %s for a car-sized model.", kQuality[quality_].estimate);
        else
            ui::hint("Custom settings (see Advanced).");
    } else {
        ui::hint("Instant preview uses idealised potential flow: no wake, no vortices. "
                 "Choose OpenFOAM CFD in the toolbar for a real simulation.");
    }
    ImGui::Checkbox("Model sits on the floor", &params_.groundPlane);
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Cars: on. Aircraft or free-flying objects: off.");

    if (ImGui::TreeNode("Advanced")) {
        bool custom = false;
        custom |= ImGui::SliderInt("Grid cells (x)", &params_.gridCellsX, 32, 256);
        if (solverKind_ == SolverKind::OpenFoam) {
            custom |= ImGui::SliderInt("Iterations", &params_.iterations, 50, 3000);
            custom |= ImGui::SliderInt("Refinement", &params_.refinementLevel, 1, 6);
            ImGui::SliderInt("CPU cores", &params_.processors, 1, 16);
        }
        ImGui::DragFloat("Upstream (x L)", &params_.upstream, 0.05f, 0.5f, 10.0f, "%.2f");
        ImGui::DragFloat("Downstream (x L)", &params_.downstream, 0.05f, 1.0f, 20.0f, "%.2f");
        ImGui::DragFloat("Side clearance (x L)", &params_.side, 0.05f, 0.5f, 10.0f, "%.2f");
        if (custom)
            quality_ = 3;

        static char dirBuf[1024] = "";
        static bool initDir = false;
        if (!initDir) {
            std::snprintf(dirBuf, sizeof dirBuf, "%s", params_.workDir.string().c_str());
            initDir = true;
        }
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##casedir", dirBuf, sizeof dirBuf))
            params_.workDir = dirBuf;
        if (ImGui::Button("Open saved result", ImVec2(-1, 0))) {
            try {
                setField(solvers::loadOpenFoamResult(dirBuf));
                fieldIsPreview_ = false;
                fieldSpeed_ = field_.freestreamSpeed;
                params_.inletSpeed = field_.freestreamSpeed;
                status_ = "Loaded saved result from " + std::string(dirBuf);
            } catch (const std::exception& e) {
                status_ = std::string("Open failed: ") + e.what();
            }
        }
        ImGui::TreePop();
    }
    ImGui::EndDisabled();

    // ---- Results --------------------------------------------------------
    if (!field_.empty()) {
        ImGui::Dummy(ImVec2(0, 6));
        ui::sectionHeader("RESULT");
        const int u = static_cast<int>(speedUnit_);
        ImGui::Text("%s", fieldIsPreview_ ? "Instant preview" : "OpenFOAM solution");
        ui::hint("Solved at %.1f %s. Peak speed %.1f %s.", speedToDisplay(fieldSpeed_, u), speedUnitName(u),
                 speedToDisplay(field_.maxSpeed(), u), speedUnitName(u));
        if (needsSolve() && !field_.empty())
            ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kWarning), "Out of date - press Simulate");
    }

    ImGui::End();
}

void App::drawRightPanel()
{
    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x - ui::kRightPanelWidth, ui::kToolbarHeight));
    ImGui::SetNextWindowSize(ImVec2(ui::kRightPanelWidth, io.DisplaySize.y - ui::kToolbarHeight - ui::kStatusHeight));
    ImGui::Begin("##right", nullptr, kPanelFlags);

    ui::sectionHeader("VIEW");
    // Layers in a fixed, meaningful order; unknown passes are appended.
    std::vector<render::RenderPass*> ordered;
    for (const auto& l : kLayers)
        if (render::RenderPass* p = findPass(l.pass))
            ordered.push_back(p);
    for (const auto& p : passes_)
        if (std::find(ordered.begin(), ordered.end(), p.get()) == ordered.end())
            ordered.push_back(p.get());

    for (render::RenderPass* p : ordered) {
        const LayerInfo* info = layerInfo(p->name());
        ImGui::PushID(p);
        ImGui::Checkbox("##on", &p->enabled);
        ImGui::SameLine();
        const bool open = ImGui::TreeNodeEx(info ? info->label : p->name(), ImGuiTreeNodeFlags_SpanAvailWidth);
        if (info && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", info->help);
        if (open) {
            ImGui::BeginDisabled(!p->enabled);
            p->drawUi();
            ImGui::EndDisabled();
            ImGui::TreePop();
        }
        ImGui::PopID();
    }

    ImGui::Dummy(ImVec2(0, 6));
    ui::sectionHeader("SCENE");
    if (ImGui::Button("Reset view (F)", ImVec2(-1, 0)))
        frameCamera();
    ImGui::ColorEdit3("Background", &background_.x, ImGuiColorEditFlags_NoInputs);
    ui::hint("Left drag: orbit   Right drag: pan   Wheel: zoom   Space: play/pause");

    ImGui::End();
}

void App::drawStatusBar()
{
    const ImGuiIO& io = ImGui::GetIO();
    const float H = ui::kStatusHeight;
    ImGui::SetNextWindowPos(ImVec2(0, io.DisplaySize.y - H));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, H));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, IM_COL32(20, 22, 26, 255));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(12, 0));
    ImGui::Begin("##status", nullptr, kPanelFlags);

    const float textH = ImGui::GetTextLineHeight();
    centreInBar(H, textH);
    if (running_) {
        core::SolverProgress p;
        {
            std::lock_guard lock(solverState_.mutex);
            p = solverState_.progress;
        }
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kAccentHover), "%s", p.stage.c_str());
        ImGui::SameLine();
        centreInBar(H, 8.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0, 0));
        ImGui::ProgressBar(p.fraction, ImVec2(260, 8), "");
        ImGui::PopStyleVar();
        ImGui::SameLine();
        centreInBar(H, textH);
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kTextMuted), "%d%%  %s",
                           static_cast<int>(p.fraction * 100.0f), p.message.c_str());
    } else {
        ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kTextMuted), "%s", status_.c_str());
    }

    char right[128];
    std::snprintf(right, sizeof right, "%s   %.0f FPS", playing_ ? "Playing" : "Paused", io.Framerate);
    const float rw = ImGui::CalcTextSize(right).x;
    ImGui::SameLine();
    ImGui::SetCursorPosX(io.DisplaySize.x - rw - 12.0f);
    centreInBar(H, textH);
    ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(ui::colour::kTextMuted), "%s", right);

    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

} // namespace app

// The slicer-style interface: toolbar, model/simulation panel on the left,
// view layers on the right, status bar at the bottom, 3D scene in between.

#include "app/App.h"

#include "app/ui/Theme.h"
#include "render/passes/SlicePass.h"
#include "solvers/openfoam/OpenFoamSolver.h"
#include "solvers/synthetic/SyntheticSolver.h"

#include <imgui.h>
#include <ImGuizmo.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>

namespace app {

namespace {

constexpr ImGuiWindowFlags kPanelFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoBringToFrontOnFocus;
// Side panels may outgrow short windows: allow a (thin, themed) scrollbar.
constexpr ImGuiWindowFlags kSidePanelFlags = kPanelFlags & ~ImGuiWindowFlags_NoScrollbar;

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

bool App::gridMatchesField() const
{
    const core::SimulationParams& a = fieldParams_;
    const core::SimulationParams& b = params_;
    return a.gridCellsX == b.gridCellsX && a.upstream == b.upstream && a.downstream == b.downstream && a.side == b.side &&
           a.groundPlane == b.groundPlane;
}

bool App::needsSolve() const
{
    if (field_.empty() || !gridMatchesField())
        return true;
    if (std::abs(fieldSpeed_ - params_.inletSpeed) > 1e-3f)
        return true;
    if (solverKind_ == SolverKind::OpenFoam)
        return fieldIsPreview_ || fieldParams_.refinementLevel != params_.refinementLevel ||
               fieldParams_.iterations != params_.iterations;
    return false;
}

void App::simulationSettingsChanged()
{
    if (field_.empty() || fieldIsPreview_) {
        previewSyntheticField(); // cheap (or cached): follow the new settings immediately
        return;
    }
    if (needsSolve() && restoreFromCache(SolverKind::OpenFoam))
        return; // these settings were already simulated
    if (needsSolve())
        status_ = "Settings changed - press Simulate to update the result (showing the previous solution)";
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
        if (!field_.empty() && fieldIsPreview_ && gridMatchesField()) {
            rescaleField(params_.inletSpeed);
            setPlaying(true);
            return;
        }
        playAfterSolve_ = true;
        previewSyntheticField();
        return;
    }
    if (restoreFromCache(SolverKind::OpenFoam)) {
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

void App::openPickedFile(const std::string& path)
{
    if (loadBody(path, kUnits[unitsPreset_].toMetres)) {
        previewSyntheticField();
        frameCamera();
        setPlaying(false);
    }
}

void App::drawUi()
{
    filePicker_.drawPopups();
    const ImGuiIO& io = ImGui::GetIO();
    updateSceneRect(io.DisplaySize.x, io.DisplaySize.y, io.DisplayFramebufferScale.x);
    drawToolbar();
    drawLeftPanel();
    drawRightPanel();
    drawStatusBar();
    drawGizmo();

    // View cube in the top-right corner of the 3D view.
    const float cubeSize = 140.0f;
    const ImVec2 cubeCentre(io.DisplaySize.x - ui::kRightPanelWidth - cubeSize * 0.75f,
                            ui::kToolbarHeight + cubeSize * 0.75f);
    const ui::ViewCubeResult cube = viewCube_.draw(cubeCentre, cubeSize, camera_.yaw, camera_.pitch);
    if (cube.clicked)
        camera_.flyTo(cube.yaw, cube.pitch);
}

void App::drawGizmo()
{
    ImGuizmo::BeginFrame();
    if (gizmo_ == Gizmo::None || body_.empty() || cfdRunning()) {
        gizmoWasUsing_ = false;
        return;
    }
    const ImGuiIO& io = ImGui::GetIO();
    const float x = ui::kLeftPanelWidth, y = ui::kToolbarHeight;
    const float w = io.DisplaySize.x - ui::kLeftPanelWidth - ui::kRightPanelWidth;
    const float h = io.DisplaySize.y - ui::kToolbarHeight - ui::kStatusHeight;
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetDrawlist(ImGui::GetBackgroundDrawList());
    ImGuizmo::SetRect(x, y, w, h);

    const glm::vec3 centre = body_.bounds().centre();
    const glm::mat4 R = glm::eulerAngleZYX(glm::radians(bodyRotation_.z), glm::radians(bodyRotation_.y),
                                           glm::radians(bodyRotation_.x));
    glm::mat4 model = glm::translate(glm::mat4(1.0f), centre) * R;
    const glm::mat4 view = camera_.view();
    const glm::mat4 proj = camera_.projection(w / std::max(h, 1.0f));

    const bool ctrl = io.KeyCtrl;
    const float rotateSnap[3] = {15.0f, 15.0f, 15.0f};
    const ImGuizmo::OPERATION op = gizmo_ == Gizmo::Move
                                       ? (params_.groundPlane ? ImGuizmo::TRANSLATE_X | ImGuizmo::TRANSLATE_Y : ImGuizmo::TRANSLATE)
                                       : ImGuizmo::ROTATE;
    ImGuizmo::SetGizmoSizeClipSpace(0.18f);
    if (ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(proj), op, ImGuizmo::WORLD, glm::value_ptr(model),
                             nullptr, ctrl && gizmo_ == Gizmo::Rotate ? rotateSnap : nullptr)) {
        const glm::vec3 moved = glm::vec3(model[3]) - centre;
        if (gizmo_ == Gizmo::Move) {
            bodyPosition_ += glm::vec2(moved);
            if (!params_.groundPlane)
                bodyHeight_ = std::max(bodyHeight_ + moved.z, 0.0f);
        } else {
            glm::mat3 r(model);
            for (int c = 0; c < 3; ++c)
                r[c] = glm::normalize(r[c]);
            float ez = 0, ey = 0, ex = 0;
            glm::extractEulerAngleZYX(glm::mat4(r), ez, ey, ex);
            bodyRotation_ = glm::degrees(glm::vec3(ex, ey, ez));
        }
        bodyTransformEdited(false);
    }
    const bool using_ = ImGuizmo::IsUsing();
    if (gizmoWasUsing_ && !using_)
        bodyTransformEdited(true); // drag finished: refresh the flow preview
    gizmoWasUsing_ = using_;
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
    ImGui::BeginDisabled(cfdRunning() || filePicker_.busy());
    if (ImGui::Button("  Open STL...  "))
        filePicker_.openDialog();
    ImGui::SameLine();
    centreInBar(H, frameH);
    if (ImGui::Button("  Samples  "))
        ImGui::OpenPopup("##samples");
    ImGui::EndDisabled();
    if (ImGui::BeginPopup("##samples")) {
        filePicker_.drawSamplesMenu();
        ImGui::EndPopup();
    }
    if (filePicker_.busy()) {
        ImGui::SameLine();
        centreInBar(H, ImGui::GetTextLineHeight());
        ImGui::TextDisabled("Opening...");
    }
    if (auto picked = filePicker_.takeResult())
        openPickedFile(*picked);

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
    ImGui::BeginDisabled(cfdRunning());
    if (ImGui::DragFloat("##speed", &shown, unit == 0 ? 0.5f : 0.1f, unit == 0 ? 1.0f : 0.3f, unit == 0 ? 600.0f : 170.0f,
                         unit == 0 ? "%.0f" : "%.1f"))
        params_.inletSpeed = std::max(speedFromDisplay(shown, unit), 0.1f);
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        if (fieldIsPreview_)
            rescaleField(params_.inletSpeed);
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
    if (running_ && !cfdRunning()) {
        playButton("Preview...", 1, ui::colour::kAccentActive, ui::colour::kAccentActive, ImVec2(playW, playH));
    } else if (running_) {
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
    ImGui::BeginDisabled(cfdRunning());
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
    ImGui::Begin("##left", nullptr, kSidePanelFlags);
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

    ImGui::BeginDisabled(cfdRunning());
    ImGui::TextUnformatted("File units");
    const char* unitLabels[] = {kUnits[0].name, kUnits[1].name, kUnits[2].name, kUnits[3].name};
    if (ui::segmented("units", &unitsPreset_, unitLabels, 4, fullW) && !bodyPath_.empty()) {
        bodyScale_ = kUnits[unitsPreset_].toMetres;
        bodyTransformEdited(true);
        frameCamera();
    }

    ImGui::Dummy(ImVec2(0, 2));
    ImGui::TextUnformatted("Up axis in file");
    const char* upLabels[] = {"+Z", "+Y", "+X"};
    bool reorient = ui::segmented("up", &upAxis_, upLabels, 3, fullW);
    if (reorient)
        bodyTransformEdited(true);
    ImGui::EndDisabled();

    // ---- Position --------------------------------------------------------
    ImGui::Dummy(ImVec2(0, 6));
    ui::sectionHeader("POSITION");
    ImGui::BeginDisabled(cfdRunning());
    {
        const char* toolLabels[] = {"Select (Q)", "Move (W)", "Rotate (E)"};
        int tool = static_cast<int>(gizmo_);
        if (ui::segmented("tool", &tool, toolLabels, 3, fullW))
            gizmo_ = static_cast<Gizmo>(tool);

        if (ImGui::Checkbox("Rests on the floor", &params_.groundPlane)) {
            if (!params_.groundPlane && bodyHeight_ <= 0.0f)
                bodyHeight_ = 0.25f * std::max(body_.bounds().size().z, 1e-3f);
            bodyTransformEdited(true);
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Cars: on (height is set automatically). Aircraft or raised objects: off.");

        const float labelW = 64.0f;
        auto row = [&](const char* label) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(label);
            ImGui::SameLine(labelW);
        };
        const float step = std::max(body_.bounds().radius(), 1e-3f) * 0.01f;

        row("X, Y (m)");
        ImGui::SetNextItemWidth(-1);
        if (ImGui::DragFloat2("##pos", &bodyPosition_.x, step, 0.0f, 0.0f, "%.3f"))
            bodyTransformEdited(false);
        if (ImGui::IsItemDeactivatedAfterEdit())
            bodyTransformEdited(true);

        row("Height");
        ImGui::BeginDisabled(params_.groundPlane);
        ImGui::SetNextItemWidth(-1);
        float shownHeight = params_.groundPlane ? 0.0f : bodyHeight_;
        if (ImGui::DragFloat("##height", &shownHeight, step, 0.0f, 1e4f, "%.3f m above floor")) {
            bodyHeight_ = std::max(shownHeight, 0.0f);
            bodyTransformEdited(false);
        }
        if (ImGui::IsItemDeactivatedAfterEdit())
            bodyTransformEdited(true);
        ImGui::EndDisabled();

        const char* axisNames[] = {"Roll X", "Pitch Y", "Yaw Z"};
        for (int a = 0; a < 3; ++a) {
            ImGui::PushID(a);
            row(axisNames[a]);
            const float btnW = 38.0f;
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - 2.0f * (btnW + ImGui::GetStyle().ItemSpacing.x));
            if (ImGui::DragFloat("##rot", &bodyRotation_[a], 0.5f, -360.0f, 360.0f, "%.1f deg"))
                bodyTransformEdited(false);
            if (ImGui::IsItemDeactivatedAfterEdit())
                bodyTransformEdited(true);
            ImGui::SameLine();
            if (ImGui::Button("-90", ImVec2(btnW, 0))) {
                bodyRotation_[a] = std::remainder(bodyRotation_[a] - 90.0f, 360.0f);
                bodyTransformEdited(true);
            }
            ImGui::SameLine();
            if (ImGui::Button("+90", ImVec2(btnW, 0))) {
                bodyRotation_[a] = std::remainder(bodyRotation_[a] + 90.0f, 360.0f);
                bodyTransformEdited(true);
            }
            ImGui::PopID();
        }
        if (ImGui::Button("Reset position", ImVec2(-1, 0))) {
            bodyRotation_ = glm::vec3(0.0f);
            bodyPosition_ = glm::vec2(0.0f);
            bodyHeight_ = 0.0f;
            params_.groundPlane = true;
            bodyTransformEdited(true);
            frameCamera();
        }
    }
    ui::hint("Wind blows along +X. Point the nose into the wind (towards -X).");
    ImGui::EndDisabled();

    // ---- Simulation -----------------------------------------------------
    ImGui::Dummy(ImVec2(0, 6));
    ui::sectionHeader("SIMULATION");
    ImGui::BeginDisabled(cfdRunning());
    if (solverKind_ == SolverKind::OpenFoam) {
        ImGui::TextUnformatted("Quality");
        const char* qLabels[] = {kQuality[0].name, kQuality[1].name, kQuality[2].name};
        int q = std::min(quality_, 2);
        if (ui::segmented("quality", &q, qLabels, 3, fullW)) {
            quality_ = q;
            applyQualityPreset();
            simulationSettingsChanged();
        }
        if (quality_ <= 2)
            ui::hint("Estimated run time %s for a car-sized model.", kQuality[quality_].estimate);
        else
            ui::hint("Custom settings (see Advanced).");
    } else {
        ui::hint("Instant preview uses idealised potential flow: no wake, no vortices. "
                 "Choose OpenFOAM CFD in the toolbar for a real simulation.");
    }

    if (ImGui::TreeNode("Advanced")) {
        bool custom = false;
        bool edited = false; // a slider was released after a change
        auto done = [&] { edited |= ImGui::IsItemDeactivatedAfterEdit(); };
        custom |= ImGui::SliderInt("Grid cells (x)", &params_.gridCellsX, 32, 256);
        done();
        if (solverKind_ == SolverKind::OpenFoam) {
            custom |= ImGui::SliderInt("Iterations", &params_.iterations, 50, 3000);
            done();
            custom |= ImGui::SliderInt("Refinement", &params_.refinementLevel, 1, 6);
            done();
            ImGui::SliderInt("CPU cores", &params_.processors, 1, 16);
        }
        ImGui::DragFloat("Upstream (x L)", &params_.upstream, 0.05f, 0.5f, 10.0f, "%.2f");
        done();
        ImGui::DragFloat("Downstream (x L)", &params_.downstream, 0.05f, 1.0f, 20.0f, "%.2f");
        done();
        ImGui::DragFloat("Side clearance (x L)", &params_.side, 0.05f, 0.5f, 10.0f, "%.2f");
        done();
        if (edited)
            simulationSettingsChanged();
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
        ui::hint("Solved at %.1f %s on a %d x %d x %d grid. Peak speed %.1f %s.", speedToDisplay(fieldSpeed_, u),
                 speedUnitName(u), field_.dims.x, field_.dims.y, field_.dims.z, speedToDisplay(field_.maxSpeed(), u),
                 speedUnitName(u));
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
    ImGui::Begin("##right", nullptr, kSidePanelFlags);

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
        if (ImGui::Checkbox("##on", &p->enabled) && p->enabled)
            ImGui::SetNextItemOpen(true); // show a layer's settings when it is switched on
        ImGui::SameLine();
        const bool open = ImGui::TreeNodeEx(info ? info->label : p->name(), ImGuiTreeNodeFlags_SpanAvailWidth);
        if (info && ImGui::IsItemHovered())
            ImGui::SetTooltip("%s", info->help);
        if (std::strcmp(p->name(), "Streamline") == 0 && p->enabled) {
            const auto* slice = dynamic_cast<const render::SlicePass*>(findPass("Slice"));
            if (slice && slice->ownsStreamlines()) {
                ImGui::SameLine();
                ImGui::TextDisabled("(on section)");
            }
        }
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
    ui::hint("Left drag: orbit   Right drag: pan   Wheel: zoom   Space: play/pause   F11: fullscreen");

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

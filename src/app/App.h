#pragma once

#include "app/Options.h"
#include "app/ui/FilePicker.h"
#include "app/ui/ViewCube.h"
#include "core/FlowField.h"
#include "core/ISolver.h"
#include "core/SimulationParams.h"
#include "core/SurfaceMesh.h"
#include "render/FlowTextures.h"
#include "render/OrbitCamera.h"
#include "render/RenderPass.h"

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct GLFWwindow;

namespace app {

class App {
public:
    explicit App(Options options);
    ~App();
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    // Window mode: runs until the window closes. Screenshot mode: renders
    // options.frames frames offscreen, writes the PNG and returns.
    int run();

private:
    enum class SolverKind { Synthetic = 0, OpenFoam = 1, Lbm = 2 };
    static bool isCfd(SolverKind kind) { return kind != SolverKind::Synthetic; }
    static bool solverAvailable(SolverKind kind);
    static const char* solverLabel(SolverKind kind); // e.g. for the solver menu and the result panel

    void initWindow();
    void initImGui();
    void createPasses();

    // Scene changes (main thread only).
    bool loadBody(const std::string& path, float scale); // empty path = test sphere
    void applyBodyTransform(); // rawBody_ -> body_ (scale, up preset, rotation, position)
    void bodyTransformEdited(bool finished); // live update while editing, preview when finished
    void drawGizmo();
    void previewSyntheticField(); // async in window mode, synchronous for screenshots
    void rescaleField(float newSpeed); // potential flow scales exactly with U
    void setField(core::FlowField field);
    // Show the unsteady clip (if the field has one) in the flow texture,
    // stepping through it while playing; restores the mean when switched off.
    void advanceClip(float dt);
    void notifyBody();
    void notifyField();
    void frameCamera();

    // Solver worker.
    void startSolver(SolverKind kind);
    void cancelSolver();
    void pollSolver(); // adopt a finished result, surface errors

    // Scene rendering into a sub-rectangle of the framebuffer (GL pixels, origin bottom-left).
    void renderScene(int x, int y, int width, int height, float time, float dt);
    void handleCameraInput();

    // UI (AppUi.cpp).
    void drawUi();
    void drawToolbar();
    void drawLeftPanel();
    void drawRightPanel();
    void drawStatusBar();
    void updateSceneRect(float displayW, float displayH, float fbScale);

    // Play / solve workflow.
    void onPlayPressed();
    bool needsSolve() const;
    bool gridMatchesField() const; // same output grid and tunnel as the current field
    void simulationSettingsChanged(); // refresh the preview, or flag a CFD result as out of date
    void setPlaying(bool playing);
    void applyQualityPreset();
    render::RenderPass* findPass(const char* name) const;
    bool cfdRunning() const { return running_ && isCfd(runningKind_); }
    void openPickedFile(const std::string& path);

    int runWindow();
    int runScreenshot();

    Options options_;
    GLFWwindow* window_ = nullptr;
    bool imguiReady_ = false;

    render::OrbitCamera camera_;
    ui::FilePicker filePicker_;
    ui::ViewCube viewCube_;
    std::vector<std::unique_ptr<render::RenderPass>> passes_;

    std::string bodyPath_;
    float bodyScale_ = 1.0f;
    core::SurfaceMesh rawBody_; // as loaded from the STL
    core::SurfaceMesh body_;    // metres, oriented (+x flow, +z up), centred in y, on z = 0
    int upAxis_ = 0;                    // see Options::upAxis
    glm::vec3 bodyRotation_{0.0f};      // degrees about world X, Y, Z (applied X, then Y, then Z)
    glm::vec2 bodyPosition_{0.0f};      // body centre in the floor plane (m)
    float bodyHeight_ = 0.0f;           // lowest point above the floor when not resting on it (m)
    enum class Gizmo { None = 0, Move = 1, Rotate = 2 };
    Gizmo gizmo_ = Gizmo::None;
    bool gizmoWasUsing_ = false;
    core::FlowField field_;
    render::FlowTextures flowTextures_;
    core::SimulationParams params_;
    glm::vec3 background_{0.075f, 0.085f, 0.10f};
    std::string status_;

    // Workflow state.
    enum class SpeedUnit { Kmh = 0, Ms = 1 };
    SpeedUnit speedUnit_ = SpeedUnit::Kmh;
    bool playing_ = false;
    bool showClip_ = true;     // animate the field's clip instead of showing its time average
    float clipTime_ = 0.0f;    // playback position, seconds
    int clipFrame_ = -1;       // clip frame in the flow texture; -1 = time average
    static constexpr float kClipFps = 24.0f;
    bool playAfterSolve_ = false;
    bool fieldIsPreview_ = true; // current field is the instant analytic preview
    SolverKind fieldKind_ = SolverKind::Synthetic; // solver that produced the current field
    float fieldSpeed_ = 0.0f;    // inlet speed (m/s) the current field was computed for
    float solveSpeed_ = 0.0f;    // inlet speed of the run in flight
    core::SimulationParams fieldParams_; // settings the current field was computed with
    core::SimulationParams solveParams_; // settings of the run in flight

    // In-memory cache of computed fields (previews and CFD results), so going
    // back to settings that were already solved is instant. Keyed by the body
    // placement (bodyVersion_) and the settings that change the result.
    struct CachedField {
        SolverKind kind;
        std::uint64_t bodyVersion;
        core::SimulationParams params;
        core::FlowField field;
    };
    static constexpr std::size_t kFieldCacheSize = 8;
    std::deque<CachedField> fieldCache_;
    std::uint64_t bodyVersion_ = 0;      // bumped whenever the body geometry/placement changes
    std::uint64_t solveBodyVersion_ = 0; // body version of the run in flight
    bool restoreFromCache(SolverKind kind);
    void storeInCache(SolverKind kind, std::uint64_t bodyVersion, const core::SimulationParams& params,
                      const core::FlowField& field);

    // Fullscreen toggle (F11): windowed placement to restore.
    bool fullscreen_ = false;
    int windowedX_ = 0, windowedY_ = 0, windowedW_ = 1600, windowedH_ = 900;
    void toggleFullscreen();
    SolverKind runningKind_ = SolverKind::Synthetic;
    bool layersShownOnPlay_ = false;
    int quality_ = 1;            // 0 draft, 1 normal, 2 fine, 3 custom
    int unitsPreset_ = 0;        // STL units: m, cm, mm, in
    struct Rect {
        int x = 0, y = 0, w = 1, h = 1;
    } sceneRect_;                // GL pixels

    // Solver state shared with the worker thread.
    struct SolverState {
        std::mutex mutex;
        core::SolverProgress progress;
        bool finished = false;
        std::string error;
        core::FlowField result;
    };
    SolverKind solverKind_ = SolverKind::Synthetic;
    std::unique_ptr<core::ISolver> solver_;
    std::thread worker_;
    std::atomic<bool> cancel_{false};
    std::atomic<bool> running_{false};
    SolverState solverState_;

    // Camera drag state.
    glm::dvec2 lastCursor_{0.0};
    bool dragging_ = false;
};

} // namespace app

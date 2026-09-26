#pragma once

#include "app/Options.h"
#include "core/FlowField.h"
#include "core/ISolver.h"
#include "core/SimulationParams.h"
#include "core/SurfaceMesh.h"
#include "render/FlowTextures.h"
#include "render/OrbitCamera.h"
#include "render/RenderPass.h"

#include <atomic>
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
    enum class SolverKind { Synthetic = 0, OpenFoam = 1 };

    void initWindow();
    void initImGui();
    void createPasses();

    // Scene changes (main thread only).
    bool loadBody(const std::string& path, float scale); // empty path = test sphere
    void applyBodyTransform(); // rawBody_ -> body_ (scale, orientation, placed on the floor)
    void previewSyntheticField();
    void setField(core::FlowField field);
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
    void setPlaying(bool playing);
    void applyQualityPreset();
    render::RenderPass* findPass(const char* name) const;

    int runWindow();
    int runScreenshot();

    Options options_;
    GLFWwindow* window_ = nullptr;
    bool imguiReady_ = false;

    render::OrbitCamera camera_;
    std::vector<std::unique_ptr<render::RenderPass>> passes_;

    std::string bodyPath_;
    float bodyScale_ = 1.0f;
    core::SurfaceMesh rawBody_; // as loaded from the STL
    core::SurfaceMesh body_;    // metres, oriented (+x flow, +z up), centred in y, on z = 0
    int upAxis_ = 0;            // see Options::upAxis
    int yawSteps_ = 0;          // quarter turns about +z
    core::FlowField field_;
    render::FlowTextures flowTextures_;
    core::SimulationParams params_;
    glm::vec3 background_{0.075f, 0.085f, 0.10f};
    std::string status_;

    // Workflow state.
    enum class SpeedUnit { Kmh = 0, Ms = 1 };
    SpeedUnit speedUnit_ = SpeedUnit::Kmh;
    bool playing_ = false;
    bool playAfterSolve_ = false;
    bool fieldIsPreview_ = true; // current field is the instant analytic preview
    float fieldSpeed_ = 0.0f;    // inlet speed (m/s) the current field was computed for
    float solveSpeed_ = 0.0f;    // inlet speed of the run in flight
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

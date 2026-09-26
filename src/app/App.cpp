#include "app/App.h"

#include "app/ui/Theme.h"

#include "core/TunnelDomain.h"
#include "io/StlLoader.h"
#include "render/passes/DomainPass.h"
#include "render/passes/ModelPass.h"
#include "render/passes/ParticlePass.h"
#include "render/passes/SlicePass.h"
#include "render/passes/StreamlinePass.h"
#include "render/passes/VortexPass.h"
#include "solvers/openfoam/OpenFoamSolver.h"
#include "solvers/synthetic/SyntheticSolver.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <stb_image_write.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace app {

namespace {

void glfwErrorCallback(int code, const char* description)
{
    std::cerr << "GLFW error " << code << ": " << description << '\n';
}

void APIENTRY glDebugCallback(GLenum, GLenum type, GLuint id, GLenum severity, GLsizei, const GLchar* message, const void*)
{
    if (severity == GL_DEBUG_SEVERITY_NOTIFICATION)
        return;
    if (type == GL_DEBUG_TYPE_PERFORMANCE && severity != GL_DEBUG_SEVERITY_HIGH)
        return; // driver hints (e.g. buffer migration for ImGui's streaming buffers)
    std::cerr << "GL " << (type == GL_DEBUG_TYPE_ERROR ? "ERROR" : "debug") << " [" << id << "]: " << message << '\n';
}

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool passSelected(const std::vector<std::string>& wanted, const char* passName)
{
    const std::string n = lower(passName);
    return std::any_of(wanted.begin(), wanted.end(), [&](const std::string& w) {
        const std::string lw = lower(w);
        return lw == n || lw == n + "s";
    });
}

App* fromWindow(GLFWwindow* w) { return static_cast<App*>(glfwGetWindowUserPointer(w)); }

} // namespace

App::App(Options options) : options_(std::move(options))
{
    params_.workDir = std::filesystem::path(WT_PROJECT_DIR) / "cases" / "run";
    // OpenFOAM scales with physical cores; hardware_concurrency counts SMT threads.
    params_.processors = static_cast<int>(std::clamp(std::thread::hardware_concurrency() / 2u, 1u, 8u));
    params_.inletSpeed = 100.0f / 3.6f; // 100 km/h
    applyQualityPreset();
    initWindow();
    if (!options_.screenshot || options_.showUi)
        initImGui();
    createPasses();

    upAxis_ = options_.upAxis;
    yawSteps_ = options_.yawSteps;
    if (!loadBody(options_.stlPath.value_or(""), options_.stlScale))
        throw std::runtime_error(status_);

    const std::string& f = options_.field;
    if (f == "synthetic") {
        previewSyntheticField();
    } else if (f.rfind("openfoam:", 0) == 0) {
        setField(solvers::loadOpenFoamResult(f.substr(9)));
        fieldIsPreview_ = false;
        fieldSpeed_ = field_.freestreamSpeed;
        params_.inletSpeed = field_.freestreamSpeed;
    } else if (f != "none") {
        throw std::runtime_error("Unknown --field value: " + f);
    }

    frameCamera();
    if (options_.yawDeg)
        camera_.yaw = glm::radians(*options_.yawDeg);
    if (options_.pitchDeg)
        camera_.pitch = glm::radians(*options_.pitchDeg);
    camera_.zoom(1.0f / std::max(options_.zoom, 1e-3f));

    solverKind_ = solvers::OpenFoamSolver::available() ? SolverKind::OpenFoam : SolverKind::Synthetic;
    if (options_.solve) {
        if (lower(*options_.solve) == "openfoam")
            solverKind_ = SolverKind::OpenFoam;
        else if (lower(*options_.solve) == "synthetic")
            solverKind_ = SolverKind::Synthetic;
        else
            throw std::runtime_error("Unknown --solve value: " + *options_.solve);
    }
}

App::~App()
{
    cancelSolver();
    passes_.clear();
    flowTextures_.clear();
    if (imguiReady_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
    if (window_)
        glfwDestroyWindow(window_);
    glfwTerminate();
}

void App::initWindow()
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
        throw std::runtime_error("Failed to initialise GLFW");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    if (options_.screenshot)
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

    window_ = glfwCreateWindow(options_.width, options_.height, "Wind Tunnel", nullptr, nullptr);
    if (!window_)
        throw std::runtime_error("Failed to create an OpenGL 4.6 window");
    glfwMakeContextCurrent(window_);
    if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)))
        throw std::runtime_error("Failed to load OpenGL functions with glad");
    glfwSwapInterval(1);

    glEnable(GL_DEBUG_OUTPUT);
    glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
    glDebugMessageCallback(glDebugCallback, nullptr);
    std::cout << "OpenGL " << glGetString(GL_VERSION) << " on " << glGetString(GL_RENDERER) << '\n';

    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, [](GLFWwindow* w, double, double dy) {
        App* app = fromWindow(w);
        if (app->imguiReady_ && ImGui::GetIO().WantCaptureMouse)
            return;
        app->camera_.zoom(dy > 0 ? 0.9f : 1.0f / 0.9f);
    });
    glfwSetKeyCallback(window_, [](GLFWwindow* w, int key, int, int action, int) {
        App* app = fromWindow(w);
        if (app->imguiReady_ && ImGui::GetIO().WantCaptureKeyboard)
            return;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(w, GLFW_TRUE);
        if (key == GLFW_KEY_F && action == GLFW_PRESS)
            app->frameCamera();
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
            app->onPlayPressed();
    });
}

void App::initImGui()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ui::applyTheme();
    // Installs callbacks that chain to the scroll/key callbacks set above.
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 460");
    imguiReady_ = true;
}

void App::createPasses()
{
    passes_.push_back(std::make_unique<render::DomainPass>());
    passes_.push_back(std::make_unique<render::ModelPass>());
    passes_.push_back(std::make_unique<render::SlicePass>());
    passes_.push_back(std::make_unique<render::StreamlinePass>());
    passes_.push_back(std::make_unique<render::VortexPass>());
    passes_.push_back(std::make_unique<render::ParticlePass>());
    // Default view: body + streamlines. Slice and particles are one click away
    // in the Display panel; all at once is too cluttered to read.
    const std::vector<std::string> defaults = {"tunnel", "model", "streamline"};
    for (auto& p : passes_)
        p->enabled = passSelected(options_.passes.value_or(defaults), p->name());
}

bool App::loadBody(const std::string& path, float scale)
{
    try {
        core::SurfaceMesh mesh = path.empty() ? core::makeSphereMesh(glm::vec3(0.0f), 0.5f) : io::loadStl(path);
        if (mesh.empty())
            throw std::runtime_error("mesh has no triangles");
        rawBody_ = std::move(mesh);
        bodyPath_ = path;
        bodyScale_ = path.empty() ? 1.0f : scale;
        applyBodyTransform();
        status_ = "Loaded " + (path.empty() ? std::string("test sphere") : path) + " (" +
                  std::to_string(body_.triangleCount()) + " triangles)";
        return true;
    } catch (const std::exception& e) {
        status_ = std::string("Load failed: ") + e.what();
        std::cerr << status_ << '\n';
        return false;
    }
}

void App::applyBodyTransform()
{
    core::SurfaceMesh mesh = rawBody_;
    for (auto& p : mesh.positions) {
        glm::vec3 q = p * bodyScale_;
        if (upAxis_ == 1) // +y up -> +z up
            q = {q.x, -q.z, q.y};
        else if (upAxis_ == 2) // +x up -> +z up
            q = {-q.z, q.y, q.x};
        for (int i = 0; i < yawSteps_; ++i) // quarter turns about +z
            q = {-q.y, q.x, q.z};
        p = q;
    }
    // Centre in x/y and stand the body on the tunnel floor (z = 0).
    const core::Bounds b = mesh.bounds();
    mesh.transform(1.0f, glm::vec3(-b.centre().x, -b.centre().y, -b.min.z));
    mesh.computeFaceNormals();

    body_ = std::move(mesh);
    field_ = core::FlowField{};
    flowTextures_.clear();
    notifyBody();
    notifyField();
}

void App::previewSyntheticField()
{
    setField(solvers::makeSyntheticField(body_, params_));
    fieldIsPreview_ = true;
    fieldSpeed_ = params_.inletSpeed;
}

void App::setField(core::FlowField field)
{
    field_ = std::move(field);
    flowTextures_.upload(field_);
    notifyField();
}

void App::notifyBody()
{
    const render::SceneRefs refs{&body_, &field_, &flowTextures_};
    for (auto& p : passes_)
        p->onBodyChanged(refs);
}

void App::notifyField()
{
    const render::SceneRefs refs{&body_, &field_, &flowTextures_};
    for (auto& p : passes_)
        p->onFieldChanged(refs);
}

void App::frameCamera()
{
    core::Bounds b = body_.bounds();
    // Include some of the wake so the flow is in view.
    b.max.x += 0.6f * b.size().x;
    camera_.frame(b);
}

void App::startSolver(SolverKind kind)
{
    if (running_)
        return;
    if (worker_.joinable())
        worker_.join();
    solverKind_ = kind;
    if (kind == SolverKind::OpenFoam)
        solver_ = std::make_unique<solvers::OpenFoamSolver>();
    else
        solver_ = std::make_unique<solvers::SyntheticSolver>();

    {
        std::lock_guard lock(solverState_.mutex);
        solverState_.progress = {"Starting", 0.0f, ""};
        solverState_.finished = false;
        solverState_.error.clear();
        solverState_.result = core::FlowField{};
    }
    cancel_ = false;
    running_ = true;
    solveSpeed_ = params_.inletSpeed;
    status_ = "Running " + solver_->name();

    worker_ = std::thread([this, body = body_, params = params_]() {
        auto progress = [this](const core::SolverProgress& p) {
            std::lock_guard lock(solverState_.mutex);
            solverState_.progress = p;
        };
        core::FlowField result;
        std::string error;
        try {
            solver_->setup(body, params);
            solver_->run(progress, cancel_);
            if (cancel_)
                error = "Cancelled";
            else
                result = solver_->result();
        } catch (const std::exception& e) {
            error = e.what();
        }
        std::lock_guard lock(solverState_.mutex);
        solverState_.result = std::move(result);
        solverState_.error = std::move(error);
        solverState_.finished = true;
        running_ = false;
    });
}

void App::cancelSolver()
{
    cancel_ = true;
    if (worker_.joinable())
        worker_.join();
    running_ = false;
}

void App::pollSolver()
{
    core::FlowField result;
    std::string error;
    {
        std::lock_guard lock(solverState_.mutex);
        if (!solverState_.finished)
            return;
        solverState_.finished = false;
        result = std::move(solverState_.result);
        error = std::move(solverState_.error);
    }
    if (worker_.joinable())
        worker_.join();
    const bool play = playAfterSolve_;
    playAfterSolve_ = false;
    if (!error.empty()) {
        status_ = error == "Cancelled" ? "Simulation cancelled" : "Simulation failed: " + error;
        std::cerr << status_ << '\n';
        return;
    }
    if (result.empty()) {
        status_ = "Solver returned an empty field";
        return;
    }
    setField(std::move(result));
    fieldIsPreview_ = solverKind_ == SolverKind::Synthetic;
    fieldSpeed_ = solveSpeed_;
    status_ = "Simulation finished";
    if (play)
        setPlaying(true);
}

void App::renderScene(int x, int y, int width, int height, float time, float dt)
{
    glDisable(GL_SCISSOR_TEST);
    glClearColor(background_.r, background_.g, background_.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(x, y, width, height);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glDisable(GL_CULL_FACE);
    glEnable(GL_MULTISAMPLE);

    render::FrameContext frame;
    frame.view = camera_.view();
    frame.proj = camera_.projection(static_cast<float>(width) / static_cast<float>(std::max(height, 1)));
    frame.eye = camera_.eye();
    frame.viewport = {width, height};
    frame.time = time;
    frame.dt = std::clamp(dt, 1e-4f, 0.1f);

    if (flowTextures_.valid())
        flowTextures_.bind();
    for (auto& p : passes_)
        if (p->enabled)
            p->update(frame);
    for (auto& p : passes_)
        if (p->enabled) {
            if (flowTextures_.valid())
                flowTextures_.bind();
            p->draw(frame);
        }
}

void App::handleCameraInput()
{
    double x = 0, y = 0;
    glfwGetCursorPos(window_, &x, &y);
    const glm::dvec2 cursor(x, y);
    const glm::vec2 delta = glm::vec2(cursor - lastCursor_);
    lastCursor_ = cursor;

    const bool left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
                       glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    if (!left && !right) {
        dragging_ = false;
        return;
    }
    // Only start a drag outside the UI; keep it once started.
    if (!dragging_) {
        if (ImGui::GetIO().WantCaptureMouse || viewCube_.wantsMouse())
            return;
        int fbw = 0, fbh = 0, ww = 0, wh = 0;
        glfwGetFramebufferSize(window_, &fbw, &fbh);
        glfwGetWindowSize(window_, &ww, &wh);
        const double sx = ww > 0 ? static_cast<double>(fbw) / ww : 1.0;
        const double px = cursor.x * sx, py = fbh - cursor.y * sx; // GL pixels, bottom-left origin
        if (px < sceneRect_.x || px > sceneRect_.x + sceneRect_.w || py < sceneRect_.y || py > sceneRect_.y + sceneRect_.h)
            return;
        dragging_ = true;
        return;
    }
    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window_, &fbw, &fbh);
    if (left)
        camera_.orbit(-delta.x * 0.006f, delta.y * 0.006f);
    else
        camera_.pan(delta, static_cast<float>(fbh));
}

int App::runWindow()
{
    glfwShowWindow(window_);
    if (options_.solve) {
        playAfterSolve_ = options_.play;
        startSolver(solverKind_);
    } else if (options_.play) {
        setPlaying(true);
    }
    double last = glfwGetTime();
    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();
        pollSolver();

        const double now = glfwGetTime();
        const float dt = static_cast<float>(now - last);
        last = now;
        camera_.update(dt);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        handleCameraInput();
        drawUi();
        ImGui::Render();

        int w = 0, h = 0;
        glfwGetFramebufferSize(window_, &w, &h);
        if (w > 0 && h > 0) {
            renderScene(sceneRect_.x, sceneRect_.y, sceneRect_.w, sceneRect_.h, static_cast<float>(now), dt);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        }
        glfwSwapBuffers(window_);
    }
    return 0;
}

int App::runScreenshot()
{
    if (options_.play && !options_.solve)
        setPlaying(true);
    if (options_.solve) {
        playAfterSolve_ = options_.play;
        startSolver(solverKind_);
        std::string lastMsg;
        while (running_) {
            {
                std::lock_guard lock(solverState_.mutex);
                const auto& p = solverState_.progress;
                const std::string msg = p.stage + " " + std::to_string(static_cast<int>(p.fraction * 100)) + "% " + p.message;
                if (msg != lastMsg)
                    std::cout << msg << '\n';
                lastMsg = msg;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        pollSolver();
        std::cout << status_ << '\n';
    }

    const int w = options_.width, h = options_.height;
    // Multisampled offscreen target, resolved into a single-sample one for readback.
    GLuint fbo[2] = {0, 0}, rb[3] = {0, 0, 0};
    glCreateFramebuffers(2, fbo);
    glCreateRenderbuffers(3, rb);
    glNamedRenderbufferStorageMultisample(rb[0], 4, GL_RGBA8, w, h);
    glNamedRenderbufferStorageMultisample(rb[1], 4, GL_DEPTH_COMPONENT24, w, h);
    glNamedRenderbufferStorage(rb[2], GL_RGBA8, w, h);
    glNamedFramebufferRenderbuffer(fbo[0], GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[0]);
    glNamedFramebufferRenderbuffer(fbo[0], GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rb[1]);
    glNamedFramebufferRenderbuffer(fbo[1], GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[2]);
    if (glCheckNamedFramebufferStatus(fbo[0], GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        throw std::runtime_error("Offscreen framebuffer incomplete");

    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < options_.frames; ++i) {
        const float t = static_cast<float>(i) * dt;
        camera_.update(dt);
        if (!options_.showUi) {
            renderScene(0, 0, w, h, t, dt);
            continue;
        }
        // Drive ImGui exactly like the window loop so the panels are laid out.
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::GetIO().DisplaySize = ImVec2(static_cast<float>(w), static_cast<float>(h));
        ImGui::GetIO().DisplayFramebufferScale = ImVec2(1.0f, 1.0f);
        ImGui::NewFrame();
        drawUi();
        ImGui::Render();
        renderScene(sceneRect_.x, sceneRect_.y, sceneRect_.w, sceneRect_.h, t, dt);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
    glBlitNamedFramebuffer(fbo[0], fbo[1], 0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);

    std::vector<unsigned char> pixels(static_cast<std::size_t>(w) * h * 4);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[1]);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(2, fbo);
    glDeleteRenderbuffers(3, rb);

    // GL rows start at the bottom; PNG rows start at the top.
    stbi_flip_vertically_on_write(1);
    if (!stbi_write_png(options_.screenshot->c_str(), w, h, 4, pixels.data(), w * 4))
        throw std::runtime_error("Failed to write " + *options_.screenshot);
    std::cout << "Wrote " << *options_.screenshot << " (" << w << "x" << h << ", " << options_.frames << " frames)\n";
    return 0;
}

int App::run()
{
    return options_.screenshot ? runScreenshot() : runWindow();
}

} // namespace app

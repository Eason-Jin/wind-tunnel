#include "solvers/openfoam/OpenFoamSolver.h"

#include "core/TunnelDomain.h"
#include "core/Voxelize.h"
#include "io/StlLoader.h"
#include "solvers/openfoam/FoamCase.h"
#include "solvers/openfoam/FoamFieldReader.h"
#include "solvers/openfoam/FoamProcess.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace solvers {

namespace {

constexpr const char* kMetaFile = "windtunnel_result.txt";
constexpr const char* kSolidFile = "windtunnel_solid.bin";
constexpr const char* kTimingFile = "log.windtunnel";
constexpr const char* kGridDir = "grid";

// ---------------------------------------------------------------------------
// Work directory safety and cleanup

fs::path safeWorkDir(const fs::path& requested)
{
    if (requested.empty())
        throw std::runtime_error("OpenFOAM: work directory is empty");
    std::error_code ec;
    fs::path dir = fs::weakly_canonical(fs::absolute(requested), ec);
    if (ec)
        dir = fs::absolute(requested).lexically_normal();
    if (dir.empty() || dir == dir.root_path())
        throw std::runtime_error("OpenFOAM: refusing to use '" + dir.string() + "' as the work directory");
    if (const char* homeEnv = std::getenv("HOME"); homeEnv && *homeEnv) {
        const fs::path home = fs::weakly_canonical(fs::path(homeEnv), ec);
        // Refuse home itself and any of its ancestors (e.g. /home).
        const auto rel = home.lexically_relative(dir);
        if (!home.empty() && !rel.empty() && *rel.begin() != "..")
            throw std::runtime_error("OpenFOAM: refusing to use '" + dir.string() + "' (home directory or above) as the work directory");
    }
    if (fs::exists(dir, ec) && !fs::is_directory(dir, ec))
        throw std::runtime_error("OpenFOAM: work directory '" + dir.string() + "' is not a directory");
    return dir;
}

bool isTimeName(const std::string& name)
{
    if (name.empty())
        return false;
    char* end = nullptr;
    std::strtod(name.c_str(), &end);
    return end && *end == '\0' && name.find_first_not_of("0123456789.eE+-") == std::string::npos;
}

// Remove only what an OpenFOAM case / this solver puts in a case directory.
// Everything is a direct child of `dir`; nothing outside it is touched.
void cleanCaseDir(const fs::path& dir)
{
    std::error_code ec;
    if (!fs::exists(dir, ec))
        return;
    static const char* const kNames[] = {"0.orig", "constant", "system", kGridDir, "postProcessing", "dynamicCode", "VTK",
                                         kMetaFile, kSolidFile};
    std::vector<fs::path> doomed;
    for (const auto& entry : fs::directory_iterator(dir)) {
        const std::string name = entry.path().filename().string();
        bool match = name.rfind("processor", 0) == 0 || name.rfind("log.", 0) == 0 ||
                     (isTimeName(name) && entry.is_directory(ec));
        for (const char* n : kNames)
            match = match || name == n;
        if (match)
            doomed.push_back(entry.path());
    }
    for (const auto& p : doomed) {
        if (p.parent_path() != dir) // paranoia: only direct children
            continue;
        fs::remove_all(p);
    }
}

std::string latestTimeDir(const fs::path& caseDir)
{
    double best = -1.0;
    std::string bestName;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(caseDir, ec)) {
        const std::string name = entry.path().filename().string();
        if (!entry.is_directory() || !isTimeName(name))
            continue;
        const double t = std::strtod(name.c_str(), nullptr);
        if (t > best) {
            best = t;
            bestName = name;
        }
    }
    return bestName;
}

// ---------------------------------------------------------------------------
// Result files

struct ResultMeta {
    glm::ivec3 dims{0};
    glm::vec3 origin{0.0f};
    glm::vec3 spacing{1.0f};
    float freestream = 1.0f;
    std::string velocity; // relative to workDir
    std::string pressure;
    std::string solid;
};

std::string fmt(float v)
{
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.9g", static_cast<double>(v));
    return buf;
}

void writeMeta(const fs::path& dir, const ResultMeta& m)
{
    std::ofstream out(dir / kMetaFile, std::ios::trunc);
    out << "# windtunnel OpenFOAM result (uniform grid, cell-centred, x fastest)\n"
        << "version 1\n"
        << "dims " << m.dims.x << " " << m.dims.y << " " << m.dims.z << "\n"
        << "origin " << fmt(m.origin.x) << " " << fmt(m.origin.y) << " " << fmt(m.origin.z) << "\n"
        << "spacing " << fmt(m.spacing.x) << " " << fmt(m.spacing.y) << " " << fmt(m.spacing.z) << "\n"
        << "freestream " << fmt(m.freestream) << "\n"
        << "velocity " << m.velocity << "\n"
        << "pressure " << m.pressure << "\n"
        << "solid " << m.solid << "\n";
    if (!out)
        throw std::runtime_error("OpenFOAM: cannot write " + (dir / kMetaFile).string());
}

ResultMeta readMeta(const fs::path& dir)
{
    std::ifstream in(dir / kMetaFile);
    if (!in)
        throw std::runtime_error("No OpenFOAM result in '" + dir.string() + "' (missing " + kMetaFile + ")");
    ResultMeta m;
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string key;
        if (!(ss >> key) || key[0] == '#')
            continue;
        if (key == "dims")
            ss >> m.dims.x >> m.dims.y >> m.dims.z;
        else if (key == "origin")
            ss >> m.origin.x >> m.origin.y >> m.origin.z;
        else if (key == "spacing")
            ss >> m.spacing.x >> m.spacing.y >> m.spacing.z;
        else if (key == "freestream")
            ss >> m.freestream;
        else if (key == "velocity")
            ss >> m.velocity;
        else if (key == "pressure")
            ss >> m.pressure;
        else if (key == "solid")
            ss >> m.solid;
    }
    if (m.dims.x <= 0 || m.dims.y <= 0 || m.dims.z <= 0 || m.velocity.empty() || m.pressure.empty())
        throw std::runtime_error("OpenFOAM result metadata in '" + dir.string() + "' is incomplete");
    return m;
}

bool isUnmapped(const glm::vec3& u) { return !(std::abs(u.x) < 0.5f * foam::kUnmappedSentinel) || !std::isfinite(u.y) || !std::isfinite(u.z); }

// Fill field velocity/pressure from the mapped grid-case files. Cells in
// `solid` (and any unmapped cell) get zero velocity and pressure.
void fillField(core::FlowField& f, const fs::path& uFile, const fs::path& pFile)
{
    const std::size_t n = f.cellCount();
    std::vector<glm::vec3> U = foam::readVectorField(uFile, n);
    std::vector<float> p = foam::readScalarField(pFile, n);
    for (std::size_t c = 0; c < n; ++c) {
        if (isUnmapped(U[c]))
            f.solid[c] = 1;
        if (f.solid[c]) {
            f.velocity[c] = glm::vec3(0.0f);
            f.pressure[c] = 0.0f;
        } else {
            f.velocity[c] = U[c];
            f.pressure[c] = p[c];
        }
    }
}

// ---------------------------------------------------------------------------
// Progress helpers

// Tracks simpleFoam's "Time = N" lines and the latest initial residuals.
class SolveMonitor {
public:
    explicit SolveMonitor(int iterations) : iterations_(std::max(1, iterations)) {}

    // Returns true when a new iteration started (a good moment to report).
    bool feed(const std::string& line)
    {
        if (line.rfind("Time = ", 0) == 0) {
            iteration_ = std::atoi(line.c_str() + 7);
            return true;
        }
        const auto solving = line.find("Solving for ");
        if (solving != std::string::npos) {
            const auto comma = line.find(',', solving);
            const auto init = line.find("Initial residual = ", solving);
            if (comma != std::string::npos && init != std::string::npos) {
                const std::string field = line.substr(solving + 12, comma - solving - 12);
                if (!seen_.count(field + std::to_string(iteration_))) {
                    seen_[field + std::to_string(iteration_)] = true;
                    residuals_[field] = std::strtod(line.c_str() + init + 19, nullptr);
                }
            }
        }
        return false;
    }

    int iteration() const { return iteration_; }
    float fraction() const { return std::clamp(static_cast<float>(iteration_) / static_cast<float>(iterations_), 0.0f, 1.0f); }

    std::string summary() const
    {
        std::string s = "Iteration " + std::to_string(iteration_) + "/" + std::to_string(iterations_);
        static const char* const kOrder[] = {"Ux", "Uy", "Uz", "p", "k", "omega"};
        for (const char* key : kOrder) {
            auto it = residuals_.find(key);
            if (it == residuals_.end())
                continue;
            char buf[48];
            std::snprintf(buf, sizeof buf, "  %s %.2e", key, it->second);
            s += buf;
        }
        return s;
    }

private:
    int iterations_;
    int iteration_ = 0;
    std::map<std::string, double> residuals_;
    std::map<std::string, bool> seen_;
};

double seconds(std::chrono::steady_clock::time_point since)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

} // namespace

// ---------------------------------------------------------------------------

void OpenFoamSolver::setup(const core::SurfaceMesh& body, const core::SimulationParams& params)
{
    if (body.empty())
        throw std::runtime_error("OpenFOAM: body mesh is empty");
    if (params.inletSpeed <= 0.0f || params.kinematicViscosity <= 0.0f)
        throw std::runtime_error("OpenFOAM: inlet speed and viscosity must be positive");
    body_ = body;
    params_ = params;
    field_ = core::FlowField{};
}

void OpenFoamSolver::run(const core::ProgressFn& progress, const std::atomic<bool>& cancel)
{
    field_ = core::FlowField{};
    if (body_.empty())
        throw std::runtime_error("OpenFOAM: setup() was not called");

    foam::FoamInstall install;
    if (!foam::findFoamInstall(install) || !foam::hasFoamApplication(install, "simpleFoam"))
        throw std::runtime_error("OpenFOAM: installation not found (expected /usr/lib/openfoam/openfoam2406 or $WM_PROJECT_DIR)");

    const fs::path dir = safeWorkDir(params_.workDir);
    const fs::path gridDir = dir / kGridDir;
    const int nProc = std::max(1, params_.processors);
    const bool parallel = nProc > 1;

    auto report = [&](const std::string& stage, float fraction, const std::string& message) {
        if (progress)
            progress({stage, std::clamp(fraction, 0.0f, 1.0f), message});
    };

    // Run one tool, mapping its own 0..1 progress into [f0, f1] overall.
    std::ostringstream timings;
    auto runStep = [&](const std::string& stage, const std::string& tool, const std::string& command, const fs::path& cwd,
                       float f0, float f1, const std::string& logName = {}) {
        const fs::path logPath = dir / ("log." + (logName.empty() ? tool : logName));
        report(stage, f0, "Running " + tool);
        auto lastReport = std::chrono::steady_clock::now();
        float local = 0.0f;
        foam::LineFn onLine = [&](const std::string& line) {
            // Coarse sub-progress for snappyHexMesh phases.
            if (tool == "snappyHexMesh") {
                if (line.find("Snapping phase") != std::string::npos || line.find("Morph iteration") != std::string::npos)
                    local = std::max(local, 0.6f);
                else if (line.find("Surface refinement iteration") != std::string::npos)
                    local = std::max(local, 0.1f);
                else if (line.find("Shell refinement iteration") != std::string::npos || line.find("Removing mesh beyond") != std::string::npos)
                    local = std::max(local, 0.35f);
                else if (line.find("Writing mesh") != std::string::npos)
                    local = std::max(local, 0.95f);
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - lastReport > std::chrono::milliseconds(250)) {
                lastReport = now;
                report(stage, f0 + (f1 - f0) * local, tool + ": " + line);
            }
        };
        const auto t0 = std::chrono::steady_clock::now();
        foam::runTool(install, {tool, command, cwd, logPath}, onLine, cancel);
        char buf[96];
        std::snprintf(buf, sizeof buf, "%-22s %8.1f s\n", (logName.empty() ? tool : logName).c_str(), seconds(t0));
        timings << buf;
        std::ofstream(dir / kTimingFile, std::ios::trunc) << timings.str();
    };
    const std::string mpi = "mpirun -np " + std::to_string(nProc) + " ";

    try {
        // --- Case setup -----------------------------------------------------
        report("Meshing", 0.0f, "Writing case");
        fs::create_directories(dir);
        cleanCaseDir(dir);

        const core::Bounds bodyBounds = body_.bounds();
        const core::TunnelDomain domain = core::makeTunnelDomain(bodyBounds, params_);
        const foam::CaseSpec spec = foam::makeCaseSpec({bodyBounds, params_.inletSpeed, params_.kinematicViscosity, params_.groundPlane,
                                                        params_.refinementLevel, params_.iterations, nProc},
                                                       domain);
        foam::writeMainCase(dir, spec);
        fs::create_directories(dir / "constant" / "triSurface");
        io::writeStl(body_, dir / "constant" / "triSurface" / "body.stl", "body");

        // --- Meshing --------------------------------------------------------
        runStep("Meshing", "blockMesh", "blockMesh", dir, 0.00f, 0.02f);
        runStep("Meshing", "surfaceFeatureExtract", "surfaceFeatureExtract", dir, 0.02f, 0.03f);
        runStep("Meshing", "snappyHexMesh", "snappyHexMesh -overwrite", dir, 0.03f, 0.22f);
        fs::copy(dir / "0.orig", dir / "0", fs::copy_options::recursive | fs::copy_options::overwrite_existing);
        if (parallel)
            runStep("Meshing", "decomposePar", "decomposePar -force", dir, 0.22f, 0.25f);

        // --- Solving --------------------------------------------------------
        runStep("Solving", "potentialFoam", parallel ? mpi + "potentialFoam -parallel -writephi" : "potentialFoam -writephi", dir, 0.25f, 0.27f);

        {
            constexpr float f0 = 0.27f, f1 = 0.90f;
            report("Solving", f0, "Starting simpleFoam");
            const fs::path logPath = dir / "log.simpleFoam";
            SolveMonitor monitor(params_.iterations);
            auto lastReport = std::chrono::steady_clock::now();
            foam::LineFn onLine = [&](const std::string& line) {
                if (monitor.feed(line)) {
                    const auto now = std::chrono::steady_clock::now();
                    if (now - lastReport > std::chrono::milliseconds(200) || monitor.iteration() <= 1) {
                        lastReport = now;
                        report("Solving", f0 + (f1 - f0) * monitor.fraction(), monitor.summary());
                    }
                }
            };
            const auto t0 = std::chrono::steady_clock::now();
            foam::runTool(install, {"simpleFoam", parallel ? mpi + "simpleFoam -parallel" : "simpleFoam", dir, logPath}, onLine, cancel);
            char buf[96];
            std::snprintf(buf, sizeof buf, "%-22s %8.1f s  (%d iterations)\n", "simpleFoam", seconds(t0), monitor.iteration());
            timings << buf;
            report("Solving", f1, monitor.summary());
        }
        if (parallel)
            runStep("Solving", "reconstructPar", "reconstructPar -latestTime", dir, 0.90f, 0.92f);

        const std::string latest = latestTimeDir(dir);
        if (latest.empty() || latest == "0")
            throw std::runtime_error("OpenFOAM: simpleFoam wrote no solution time directory in " + dir.string());

        // --- Mapping onto the output grid -----------------------------------
        report("Mapping", 0.92f, "Writing output grid case");
        foam::writeGridCase(gridDir, domain);
        runStep("Mapping", "blockMesh", "blockMesh", gridDir, 0.92f, 0.94f, "blockMesh.grid");
        runStep("Mapping", "mapFields", "mapFields .. -sourceTime latestTime", gridDir, 0.94f, 0.99f);

        report("Mapping", 0.99f, "Reading mapped fields");
        core::FlowField f = core::makeFieldForDomain(domain);
        f.freestreamSpeed = params_.inletSpeed;
        core::voxelizeSolid(body_, f);
        fillField(f, gridDir / "0" / "U", gridDir / "0" / "p");

        {
            std::ofstream out(dir / kSolidFile, std::ios::binary | std::ios::trunc);
            out.write(reinterpret_cast<const char*>(f.solid.data()), static_cast<std::streamsize>(f.solid.size()));
            if (!out)
                throw std::runtime_error("OpenFOAM: cannot write " + (dir / kSolidFile).string());
        }
        ResultMeta meta;
        meta.dims = f.dims;
        meta.origin = f.origin;
        meta.spacing = f.spacing;
        meta.freestream = f.freestreamSpeed;
        meta.velocity = std::string(kGridDir) + "/0/U";
        meta.pressure = std::string(kGridDir) + "/0/p";
        meta.solid = kSolidFile;
        writeMeta(dir, meta);
        std::ofstream(dir / kTimingFile, std::ios::trunc) << timings.str();

        field_ = std::move(f);
        report("Done", 1.0f, "OpenFOAM solution mapped to " + std::to_string(field_.dims.x) + "x" + std::to_string(field_.dims.y) +
                                 "x" + std::to_string(field_.dims.z) + " grid (solution time " + latest + ")");
    } catch (const foam::CancelledError&) {
        field_ = core::FlowField{};
        report("Cancelled", 0.0f, "OpenFOAM run cancelled");
    }
}

bool OpenFoamSolver::available()
{
    foam::FoamInstall install;
    return foam::findFoamInstall(install) && foam::hasFoamApplication(install, "simpleFoam");
}

core::FlowField loadOpenFoamResult(const std::filesystem::path& workDir)
{
    const ResultMeta m = readMeta(workDir);
    core::FlowField f;
    f.origin = m.origin;
    f.spacing = m.spacing;
    f.freestreamSpeed = m.freestream;
    f.resize(m.dims);

    if (!m.solid.empty()) {
        std::ifstream in(workDir / m.solid, std::ios::binary);
        in.read(reinterpret_cast<char*>(f.solid.data()), static_cast<std::streamsize>(f.solid.size()));
        if (!in || in.gcount() != static_cast<std::streamsize>(f.solid.size()))
            throw std::runtime_error("OpenFOAM result: solid mask " + (workDir / m.solid).string() + " is missing or truncated");
    }
    fillField(f, workDir / m.velocity, workDir / m.pressure);
    return f;
}

} // namespace solvers

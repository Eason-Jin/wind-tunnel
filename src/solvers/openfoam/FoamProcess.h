#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>

namespace solvers::foam {

// Thrown by runTool() when the cancel flag stops a running tool.
struct CancelledError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// The OpenFOAM installation used by the solver.
struct FoamInstall {
    std::filesystem::path projectDir; // e.g. /usr/lib/openfoam/openfoam2406
    std::filesystem::path bashrc;     // projectDir/etc/bashrc
};

// Locate OpenFOAM: $WM_PROJECT_DIR if set, else the default ESI v2406 install.
// Returns false if no etc/bashrc was found.
bool findFoamInstall(FoamInstall& out);

// True if a simpleFoam binary exists in the installation (or $FOAM_APPBIN).
bool hasFoamApplication(const FoamInstall& install, const std::string& app);

struct ToolRun {
    std::string tool;          // short name used in errors, e.g. "snappyHexMesh"
    std::string command;       // shell command, run after sourcing the environment
    std::filesystem::path dir; // working directory
    std::filesystem::path log; // combined stdout/stderr is written here
};

using LineFn = std::function<void(const std::string&)>;

// Run `bash -c "source <bashrc> && cd <dir> && exec <command>"` in its own
// process group, streaming output to run.log and to onLine (one call per
// line). Returns when the tool exits with status 0. Throws std::runtime_error
// (tool name, exit status and log tail) on failure, or CancelledError if
// `cancel` became true, after killing the whole process tree and reaping the
// child. Cancellation takes at most ~1.5 s.
void runTool(const FoamInstall& install, const ToolRun& run, const LineFn& onLine, const std::atomic<bool>& cancel);

// Last `lines` lines of a text file ("" if unreadable).
std::string fileTail(const std::filesystem::path& path, int lines);

// Single-quote a string for bash.
std::string shellQuote(const std::string& s);

} // namespace solvers::foam

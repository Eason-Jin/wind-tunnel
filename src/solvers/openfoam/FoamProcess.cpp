#include "solvers/openfoam/FoamProcess.h"

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <set>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace fs = std::filesystem;

namespace solvers::foam {

namespace {

constexpr const char* kDefaultProjectDir = "/usr/lib/openfoam/openfoam2406";

// Map pid -> parent pid for every process visible in /proc.
std::map<pid_t, pid_t> processParents()
{
    std::map<pid_t, pid_t> parents;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator("/proc", ec)) {
        const std::string name = entry.path().filename().string();
        if (name.empty() || name.find_first_not_of("0123456789") != std::string::npos)
            continue;
        std::ifstream in(entry.path() / "stat");
        std::string stat;
        if (!std::getline(in, stat))
            continue;
        // Format: pid (comm) state ppid ... ; comm may contain spaces/parens.
        const auto close = stat.rfind(')');
        if (close == std::string::npos || close + 4 >= stat.size())
            continue;
        const char* p = stat.c_str() + close + 2; // state char
        char* end = nullptr;
        const long ppid = std::strtol(p + 1, &end, 10);
        if (end == p + 1)
            continue;
        parents[static_cast<pid_t>(std::atol(name.c_str()))] = static_cast<pid_t>(ppid);
    }
    return parents;
}

// All descendants of `root` (not including root).
std::set<pid_t> descendants(pid_t root)
{
    const auto parents = processParents();
    std::map<pid_t, std::vector<pid_t>> children;
    for (const auto& [pid, ppid] : parents)
        children[ppid].push_back(pid);
    std::set<pid_t> out;
    std::vector<pid_t> stack{root};
    while (!stack.empty()) {
        const pid_t p = stack.back();
        stack.pop_back();
        auto it = children.find(p);
        if (it == children.end())
            continue;
        for (pid_t c : it->second)
            if (out.insert(c).second)
                stack.push_back(c);
    }
    return out;
}

bool anyAlive(const std::set<pid_t>& pids)
{
    for (pid_t p : pids)
        if (::kill(p, 0) == 0)
            return true;
    return false;
}

// Kill the child's process group and every descendant (mpirun ranks may
// live in their own groups), then reap the child. Bounded to ~1.5 s.
void killTree(pid_t child, int& status)
{
    std::set<pid_t> tree = descendants(child);
    auto signalAll = [&](int sig) {
        ::kill(-child, sig);
        ::kill(child, sig);
        for (pid_t p : tree)
            ::kill(p, sig);
    };
    signalAll(SIGTERM);

    bool reaped = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < deadline) {
        if (!reaped && ::waitpid(child, &status, WNOHANG) == child)
            reaped = true;
        if (reaped && !anyAlive(tree))
            return;
        // Pick up late-spawned processes while the tree is still reachable.
        if (!reaped)
            for (pid_t p : descendants(child))
                if (tree.insert(p).second)
                    ::kill(p, SIGTERM);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    signalAll(SIGKILL);
    if (!reaped)
        ::waitpid(child, &status, 0);
    // Give the kernel a moment to tear down SIGKILLed grandchildren (they
    // are reparented to init, which reaps them).
    for (int i = 0; i < 25 && anyAlive(tree); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

std::string describeStatus(int status)
{
    if (WIFEXITED(status))
        return "exit code " + std::to_string(WEXITSTATUS(status));
    if (WIFSIGNALED(status))
        return std::string("killed by signal ") + std::to_string(WTERMSIG(status)) + " (" + ::strsignal(WTERMSIG(status)) + ")";
    return "status " + std::to_string(status);
}

} // namespace

bool findFoamInstall(FoamInstall& out)
{
    std::vector<fs::path> candidates;
    if (const char* env = std::getenv("WM_PROJECT_DIR"); env && *env)
        candidates.emplace_back(env);
    candidates.emplace_back(kDefaultProjectDir);
    for (const auto& dir : candidates) {
        std::error_code ec;
        if (fs::is_regular_file(dir / "etc" / "bashrc", ec)) {
            out.projectDir = dir;
            out.bashrc = dir / "etc" / "bashrc";
            return true;
        }
    }
    return false;
}

bool hasFoamApplication(const FoamInstall& install, const std::string& app)
{
    std::error_code ec;
    if (const char* bin = std::getenv("FOAM_APPBIN"); bin && *bin && fs::is_regular_file(fs::path(bin) / app, ec))
        return true;
    const fs::path platforms = install.projectDir / "platforms";
    if (!fs::is_directory(platforms, ec))
        return false;
    for (const auto& entry : fs::directory_iterator(platforms, ec))
        if (fs::is_regular_file(entry.path() / "bin" / app, ec))
            return true;
    return false;
}

std::string shellQuote(const std::string& s)
{
    std::string out = "'";
    for (char c : s) {
        if (c == '\'')
            out += "'\\''";
        else
            out += c;
    }
    return out + "'";
}

std::string fileTail(const fs::path& path, int lines)
{
    std::ifstream in(path);
    if (!in)
        return {};
    std::deque<std::string> tail;
    std::string line;
    while (std::getline(in, line)) {
        tail.push_back(line);
        if (static_cast<int>(tail.size()) > lines)
            tail.pop_front();
    }
    std::string out;
    for (const auto& l : tail)
        out += l + '\n';
    return out;
}

void runTool(const FoamInstall& install, const ToolRun& run, const LineFn& onLine, const std::atomic<bool>& cancel)
{
    if (cancel)
        throw CancelledError("Cancelled before " + run.tool);

    std::ofstream log(run.log, std::ios::trunc);
    if (!log)
        throw std::runtime_error("OpenFOAM: cannot write log file " + run.log.string());

    // Oversubscription is harmless for other MPIs and avoids Open MPI
    // refusing -np larger than its detected slot count.
    const std::string script = "export OMPI_MCA_rmaps_base_oversubscribe=1; source " + shellQuote(install.bashrc.string()) +
                               " >/dev/null 2>&1; cd " + shellQuote(run.dir.string()) + " && exec " + run.command;
    log << "# " << run.command << "\n# in " << run.dir.string() << "\n";
    log.flush();

    int fds[2];
    if (::pipe2(fds, O_CLOEXEC) != 0)
        throw std::runtime_error(std::string("OpenFOAM: pipe failed: ") + std::strerror(errno));

    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attr;
    posix_spawn_file_actions_init(&actions);
    posix_spawnattr_init(&attr);
    posix_spawn_file_actions_addopen(&actions, 0, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_adddup2(&actions, fds[1], 1);
    posix_spawn_file_actions_adddup2(&actions, fds[1], 2);
    sigset_t noSignals, defaults;
    sigemptyset(&noSignals);
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGPIPE);
    sigaddset(&defaults, SIGTERM);
    sigaddset(&defaults, SIGINT);
    posix_spawnattr_setsigmask(&attr, &noSignals);
    posix_spawnattr_setsigdefault(&attr, &defaults);
    posix_spawnattr_setpgroup(&attr, 0); // own process group, so kill(-pid) hits the whole job
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK | POSIX_SPAWN_SETSIGDEF);

    std::string bash = "/bin/bash", dashC = "-c", scriptArg = script;
    char* argv[] = {bash.data(), dashC.data(), scriptArg.data(), nullptr};
    pid_t pid = -1;
    const int rc = ::posix_spawn(&pid, "/bin/bash", &actions, &attr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    posix_spawnattr_destroy(&attr);
    ::close(fds[1]);
    if (rc != 0) {
        ::close(fds[0]);
        throw std::runtime_error("OpenFOAM: cannot start " + run.tool + ": " + std::strerror(rc));
    }

    const int readFd = fds[0];
    std::string pending;
    char buf[65536];
    int status = 0;
    bool exited = false;
    std::chrono::steady_clock::time_point exitTime;

    auto emitLines = [&](bool flushAll) {
        std::size_t start = 0;
        for (;;) {
            const std::size_t nl = pending.find_first_of("\n\r", start);
            if (nl == std::string::npos)
                break;
            if (nl > start && onLine)
                onLine(pending.substr(start, nl - start));
            start = nl + 1;
        }
        pending.erase(0, start);
        if (flushAll && !pending.empty()) {
            if (onLine)
                onLine(pending);
            pending.clear();
        }
    };

    try {
        for (;;) {
            if (cancel) {
                killTree(pid, status);
                ::close(readFd);
                log << "\n# Cancelled\n";
                throw CancelledError("Cancelled during " + run.tool);
            }
            pollfd pfd{readFd, POLLIN, 0};
            const int pr = ::poll(&pfd, 1, 50);
            bool eof = false;
            if (pr > 0) {
                const ssize_t n = ::read(readFd, buf, sizeof buf);
                if (n > 0) {
                    log.write(buf, n);
                    pending.append(buf, static_cast<std::size_t>(n));
                    emitLines(false);
                } else if (n == 0 || (n < 0 && errno != EINTR && errno != EAGAIN)) {
                    eof = true;
                }
            }
            if (!exited && ::waitpid(pid, &status, WNOHANG) == pid) {
                exited = true;
                exitTime = std::chrono::steady_clock::now();
            }
            // A detached grandchild could keep the pipe open; don't wait on it
            // for more than a moment once the tool itself has exited.
            if (eof || (exited && std::chrono::steady_clock::now() - exitTime > std::chrono::seconds(2)))
                break;
        }
    } catch (const CancelledError&) {
        throw;
    } catch (...) {
        // onLine threw: stop the tool before propagating.
        if (!exited)
            killTree(pid, status);
        ::close(readFd);
        throw;
    }
    ::close(readFd);
    emitLines(true);
    if (!exited)
        ::waitpid(pid, &status, 0);
    log.flush();
    log.close();

    if (!(WIFEXITED(status) && WEXITSTATUS(status) == 0)) {
        throw std::runtime_error("OpenFOAM: " + run.tool + " failed (" + describeStatus(status) + "). Last lines of " +
                                 run.log.string() + ":\n" + fileTail(run.log, 25));
    }
}

} // namespace solvers::foam

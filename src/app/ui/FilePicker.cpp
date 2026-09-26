#include "app/ui/FilePicker.h"

#include <imgui.h>

#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <system_error>

extern char** environ;

namespace app::ui {

namespace {

namespace fs = std::filesystem;

// Where the app's compile definition points; the coordinator's real build
// keeps this pointed at the original repo root even when this file is built
// from a worktree copy.
fs::path projectDir()
{
#ifdef WT_PROJECT_DIR
    return fs::path(WT_PROJECT_DIR);
#else
    return fs::current_path();
#endif
}

fs::path samplesCacheDir() { return projectDir() / "cases" / "samples"; }

// The OpenFOAM tutorial geometry folder: prefer $FOAM_TUTORIALS if set.
fs::path geometryDir()
{
    if (const char* env = std::getenv("FOAM_TUTORIALS")) {
        fs::path p = fs::path(env) / "resources" / "geometry";
        std::error_code ec;
        if (fs::exists(p, ec))
            return p;
    }
    return "/usr/lib/openfoam/openfoam2406/tutorials/resources/geometry";
}

// Finds an executable on $PATH without spawning a process.
std::string findExecutable(const char* name)
{
    const char* path = std::getenv("PATH");
    if (!path)
        return {};
    std::stringstream ss{std::string(path)};
    std::string dir;
    while (std::getline(ss, dir, ':')) {
        if (dir.empty())
            continue;
        fs::path candidate = fs::path(dir) / name;
        std::error_code ec;
        if (fs::exists(candidate, ec) && access(candidate.c_str(), X_OK) == 0)
            return candidate.string();
    }
    return {};
}

// Strips one or two trailing extensions used by our sample files.
std::string stemOf(const fs::path& p)
{
    if (p.extension() == ".gz")
        return p.stem().stem().string();
    return p.stem().string();
}

bool hasStlExtension(const fs::path& p)
{
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".gz") {
        std::string ext2 = p.stem().extension().string();
        std::transform(ext2.begin(), ext2.end(), ext2.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return ext2 == ".stl";
    }
    return ext == ".stl";
}

// "simpleCar" -> "Simple car", "DTC-scaled" -> "DTC scaled".
std::string friendlyName(const std::string& stem)
{
    std::vector<std::string> words;
    std::string cur;
    auto flush = [&]() {
        if (!cur.empty()) {
            words.push_back(cur);
            cur.clear();
        }
    };
    for (char c : stem) {
        if (c == '-' || c == '_' || c == ' ') {
            flush();
            continue;
        }
        const bool isUpper = std::isupper(static_cast<unsigned char>(c)) != 0;
        const bool prevLower = !cur.empty() && std::islower(static_cast<unsigned char>(cur.back()));
        if (isUpper && prevLower)
            flush();
        cur += c;
    }
    flush();

    std::string out;
    for (size_t i = 0; i < words.size(); ++i) {
        std::string w = words[i];
        const bool looksLikeAcronym = w.size() > 1 &&
            std::all_of(w.begin(), w.end(), [](unsigned char c) { return !std::isalpha(c) || std::isupper(c); });
        if (!looksLikeAcronym)
            for (auto& c : w)
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (i == 0 && !w.empty())
            w[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(w[0])));
        if (i)
            out += ' ';
        out += w;
    }
    return out;
}

std::string formatSize(std::uintmax_t bytes)
{
    char buf[32];
    if (bytes >= 1024 * 1024)
        std::snprintf(buf, sizeof buf, "%.1f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    else if (bytes >= 1024)
        std::snprintf(buf, sizeof buf, "%.0f KB", static_cast<double>(bytes) / 1024.0);
    else
        std::snprintf(buf, sizeof buf, "%llu B", static_cast<unsigned long long>(bytes));
    return buf;
}

// Quotes a path for embedding in an `sh -c` string.
std::string shellQuote(const std::string& s)
{
    std::string out = "'";
    for (char c : s) {
        if (c == '\'')
            out += "'\\''";
        else
            out += c;
    }
    out += "'";
    return out;
}

} // namespace

FilePicker::FilePicker() = default;

FilePicker::~FilePicker()
{
    // Best-effort: nudge a still-open native dialog so we don't hang the
    // process on shutdown; the destructor still joins unconditionally.
    const long long pid = dialogPid_.load();
    if (dialogRunning_.load() && pid > 0)
        kill(static_cast<pid_t>(pid), SIGTERM);
    if (dialogThread_.joinable())
        dialogThread_.join();
    if (decompressThread_.joinable())
        decompressThread_.join();
}

bool FilePicker::busy() const { return dialogRunning_.load() || decompressRunning_.load(); }

// --- Native dialog -----------------------------------------------------

void FilePicker::openDialog()
{
    if (busy())
        return;

    std::string exe = findExecutable("zenity");
    bool isZenity = true;
    if (exe.empty()) {
        exe = findExecutable("kdialog");
        isZenity = false;
    }
    if (exe.empty()) {
        // Neither tool is installed: fall back to the built-in browser.
        openBrowser();
        return;
    }

    if (dialogThread_.joinable())
        dialogThread_.join();
    dialogRunning_ = true;
    {
        std::lock_guard lock(dialogMutex_);
        dialogHasOutput_ = false;
    }
    std::string startDir = lastDir_;
    dialogThread_ = std::thread([this, exe, isZenity, startDir]() { runNativeDialog(exe, isZenity, startDir); });
}

void FilePicker::runNativeDialog(std::string exe, bool isZenity, std::string startDir)
{
    int pipefd[2];
    std::string result;

    if (pipe(pipefd) == 0) {
        std::vector<std::string> argStorage;
        argStorage.push_back(exe);
        if (isZenity) {
            argStorage.push_back("--file-selection");
            argStorage.push_back("--title=Open STL");
            argStorage.push_back("--file-filter=STL files | *.stl *.STL");
            argStorage.push_back("--file-filter=All files | *");
            if (!startDir.empty())
                argStorage.push_back("--filename=" + startDir + "/");
        } else {
            argStorage.push_back("--getopenfilename");
            argStorage.push_back(startDir.empty() ? std::string(".") : startDir);
            argStorage.push_back("*.stl *.STL|STL files");
        }
        std::vector<char*> argv;
        argv.reserve(argStorage.size() + 1);
        for (auto& s : argStorage)
            argv.push_back(s.data());
        argv.push_back(nullptr);

        posix_spawn_file_actions_t actions;
        posix_spawn_file_actions_init(&actions);
        posix_spawn_file_actions_adddup2(&actions, pipefd[1], STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&actions, pipefd[0]);
        posix_spawn_file_actions_addclose(&actions, pipefd[1]);

        pid_t pid = -1;
        const int rc = posix_spawnp(&pid, exe.c_str(), &actions, nullptr, argv.data(), environ);
        posix_spawn_file_actions_destroy(&actions);
        close(pipefd[1]);

        if (rc == 0) {
            dialogPid_ = static_cast<long long>(pid);
            char buf[4096];
            ssize_t n;
            while ((n = read(pipefd[0], buf, sizeof buf)) > 0)
                result.append(buf, static_cast<std::size_t>(n));
            close(pipefd[0]);
            int status = 0;
            waitpid(pid, &status, 0);
            const bool ok = WIFEXITED(status) && WEXITSTATUS(status) == 0;
            if (!ok)
                result.clear();
        } else {
            close(pipefd[0]);
        }
    }

    while (!result.empty() && (result.back() == '\n' || result.back() == '\r'))
        result.pop_back();

    dialogPid_ = -1;
    {
        std::lock_guard lock(dialogMutex_);
        dialogOutput_ = result;
        dialogHasOutput_ = true;
    }
    dialogRunning_ = false; // must be last: the poller treats this as "safe to join"
}

void FilePicker::pollNativeDialog()
{
    if (dialogRunning_.load())
        return;
    bool has = false;
    std::string output;
    {
        std::lock_guard lock(dialogMutex_);
        has = dialogHasOutput_;
        if (has) {
            output = dialogOutput_;
            dialogHasOutput_ = false;
        }
    }
    if (!has)
        return;
    if (dialogThread_.joinable())
        dialogThread_.join();
    if (!output.empty()) {
        pendingResult_ = output;
        lastDir_ = fs::path(output).parent_path().string();
        statusMessage_.clear();
    } else {
        statusMessage_ = "Open cancelled";
    }
}

// --- Samples -------------------------------------------------------------

const std::vector<SampleEntry>& FilePicker::listSamples()
{
    if (samplesScanned_)
        return samples_;
    samplesScanned_ = true;

    samples_.push_back(SampleEntry{"Test sphere", fs::path(), 0, false});

    std::error_code ec;
    fs::path dir = geometryDir();
    if (fs::exists(dir, ec) && fs::is_directory(dir, ec)) {
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            if (ec)
                break;
            if (!entry.is_regular_file())
                continue;
            const fs::path& p = entry.path();
            if (!hasStlExtension(p))
                continue;
            SampleEntry sample;
            sample.displayName = friendlyName(stemOf(p));
            sample.sourcePath = p;
            sample.gzipped = p.extension() == ".gz";
            std::error_code sizeEc;
            sample.sizeBytes = fs::file_size(p, sizeEc);
            samples_.push_back(std::move(sample));
        }
    }

    // Test sphere stays first; sort the rest alphabetically by display name.
    std::sort(samples_.begin() + 1, samples_.end(),
              [](const SampleEntry& a, const SampleEntry& b) { return a.displayName < b.displayName; });
    return samples_;
}

void FilePicker::requestSample(const SampleEntry& entry)
{
    if (entry.sourcePath.empty()) {
        pendingResult_ = std::string(); // test sphere
        return;
    }
    if (!entry.gzipped) {
        pendingResult_ = entry.sourcePath.string();
        return;
    }

    fs::path destDir = samplesCacheDir();
    fs::path dest = destDir / (stemOf(entry.sourcePath) + ".stl");

    std::error_code ec;
    if (fs::exists(dest, ec)) {
        auto srcTime = fs::last_write_time(entry.sourcePath, ec);
        auto dstTime = fs::last_write_time(dest, ec);
        if (!ec && dstTime >= srcTime) {
            pendingResult_ = dest.string(); // cache hit, nothing to decompress
            return;
        }
    }

    if (decompressRunning_.load())
        return; // one at a time; the menu item shows "busy" via decompressingDisplayName_
    if (decompressThread_.joinable())
        decompressThread_.join();
    decompressRunning_ = true;
    decompressingDisplayName_ = entry.displayName;
    fs::path src = entry.sourcePath;
    decompressThread_ = std::thread([this, src, dest, destDir]() { runDecompress(src, dest, destDir); });
}

void FilePicker::runDecompress(fs::path src, fs::path dest, fs::path destDir)
{
    std::error_code ec;
    fs::create_directories(destDir, ec);

    std::string resultPath;
    std::string error;
    if (ec) {
        error = "Could not create " + destDir.string() + ": " + ec.message();
    } else {
        const fs::path tmp = dest.string() + ".part";
        const std::string cmd = "gzip -dc " + shellQuote(src.string()) + " > " + shellQuote(tmp.string());
        char shPath[] = "/bin/sh";
        char cFlag[] = "-c";
        std::vector<char> cmdBuf(cmd.begin(), cmd.end());
        cmdBuf.push_back('\0');
        char* argv[] = {shPath, cFlag, cmdBuf.data(), nullptr};

        pid_t pid = -1;
        const int rc = posix_spawn(&pid, "/bin/sh", nullptr, nullptr, argv, environ);
        if (rc == 0) {
            int status = 0;
            waitpid(pid, &status, 0);
            std::error_code sizeEc;
            if (WIFEXITED(status) && WEXITSTATUS(status) == 0 && fs::exists(tmp, sizeEc) &&
                fs::file_size(tmp, sizeEc) > 0) {
                fs::rename(tmp, dest, ec);
                if (ec)
                    error = "Could not move decompressed file into place: " + ec.message();
                else
                    resultPath = dest.string();
            } else {
                error = "gzip failed to extract " + src.string();
                fs::remove(tmp, sizeEc);
            }
        } else {
            error = "Could not spawn gzip";
        }
    }

    std::lock_guard lock(decompressMutex_);
    decompressResult_ = resultPath;
    decompressError_ = error;
    decompressHasResult_ = true;
    decompressRunning_ = false;
}

void FilePicker::pollDecompress()
{
    if (decompressRunning_.load())
        return;
    bool has = false;
    std::string result, error;
    {
        std::lock_guard lock(decompressMutex_);
        has = decompressHasResult_;
        if (has) {
            result = decompressResult_;
            error = decompressError_;
            decompressHasResult_ = false;
        }
    }
    if (!has)
        return;
    if (decompressThread_.joinable())
        decompressThread_.join();
    decompressingDisplayName_.clear();
    if (!error.empty())
        statusMessage_ = error;
    else if (!result.empty())
        pendingResult_ = result;
}

void FilePicker::drawSamplesMenu()
{
    const std::vector<SampleEntry>& samples = listSamples();
    for (const SampleEntry& s : samples) {
        std::string label = s.displayName;
        if (!s.sourcePath.empty())
            label += "  (" + formatSize(s.sizeBytes) + (s.gzipped ? ", gz" : "") + ")";
        const bool isExtractingThis = decompressRunning_.load() && decompressingDisplayName_ == s.displayName;
        if (isExtractingThis)
            label += "  (extracting...)";
        ImGui::BeginDisabled(isExtractingThis || (decompressRunning_.load() && s.gzipped));
        if (ImGui::MenuItem(label.c_str()))
            requestSample(s);
        ImGui::EndDisabled();
    }
    if (!statusMessage_.empty()) {
        ImGui::Separator();
        ImGui::TextDisabled("%s", statusMessage_.c_str());
    }
}

// --- Built-in browser fallback --------------------------------------------

void FilePicker::openBrowser()
{
    if (browserDir_.empty()) {
        std::error_code ec;
        browserDir_ = fs::current_path(ec);
    }
    refreshBrowserEntries();
    browserOpen_ = true;
    browserNeedsOpen_ = true;
}

void FilePicker::navigateTo(const fs::path& dir)
{
    std::error_code ec;
    fs::path canonical = fs::weakly_canonical(dir, ec);
    if (ec || !fs::is_directory(canonical, ec))
        return;
    browserDir_ = canonical;
    refreshBrowserEntries();
}

void FilePicker::refreshBrowserEntries()
{
    browserEntries_.clear();
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(browserDir_, ec)) {
        if (ec)
            break;
        const bool isDir = entry.is_directory();
        if (!isDir && !hasStlExtension(entry.path()))
            continue;
        browserEntries_.push_back(BrowserEntry{entry.path().filename().string(), entry.path(), isDir});
    }
    std::sort(browserEntries_.begin(), browserEntries_.end(), [](const BrowserEntry& a, const BrowserEntry& b) {
        if (a.isDir != b.isDir)
            return a.isDir;
        return a.name < b.name;
    });
    std::snprintf(browserPathBuf_, sizeof browserPathBuf_, "%s", browserDir_.string().c_str());
}

void FilePicker::drawPopups()
{
    pollNativeDialog();
    pollDecompress();

    if (!browserOpen_)
        return;

    if (browserNeedsOpen_) {
        ImGui::OpenPopup("Open STL##browser");
        browserNeedsOpen_ = false;
    }

    ImGui::SetNextWindowSize(ImVec2(560, 420), ImGuiCond_FirstUseEver);
    bool open = browserOpen_;
    if (ImGui::BeginPopupModal("Open STL##browser", &open, ImGuiWindowFlags_NoSavedSettings)) {
        ImGui::SetNextItemWidth(-1);
        if (ImGui::InputText("##path", browserPathBuf_, sizeof browserPathBuf_, ImGuiInputTextFlags_EnterReturnsTrue))
            navigateTo(browserPathBuf_);
        if (ImGui::Button("Up"))
            navigateTo(browserDir_.parent_path());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", browserDir_.string().c_str());

        ImGui::Separator();
        ImGui::BeginChild("##entries", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
        for (const BrowserEntry& e : browserEntries_) {
            std::string label = (e.isDir ? "[dir]  " : "        ") + e.name;
            if (ImGui::Selectable(label.c_str(), false, ImGuiSelectableFlags_AllowDoubleClick) &&
                ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                if (e.isDir) {
                    navigateTo(e.path);
                } else {
                    pendingResult_ = e.path.string();
                    open = false;
                    ImGui::CloseCurrentPopup();
                }
            }
        }
        ImGui::EndChild();

        if (ImGui::Button("Cancel")) {
            open = false;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    browserOpen_ = open;
}

std::optional<std::string> FilePicker::takeResult()
{
    std::optional<std::string> r = std::move(pendingResult_);
    pendingResult_.reset();
    return r;
}

} // namespace app::ui

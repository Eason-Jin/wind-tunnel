#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace app::ui {

// One entry in the sample-model list: an OpenFOAM tutorial geometry file, or
// the built-in test sphere (empty sourcePath).
struct SampleEntry {
    std::string displayName;
    std::filesystem::path sourcePath; // empty = built-in test sphere
    std::uintmax_t sizeBytes = 0;
    bool gzipped = false;
};

// Opens STL files: a native OS file-selection dialog (zenity/kdialog), a
// built-in ImGui browser fallback, and a curated list of OpenFOAM tutorial
// sample geometries (transparently gunzipped and cached on first use).
//
// All slow work (spawning the dialog, decompressing a sample) happens on a
// background thread; call drawPopups() once per frame from the render loop
// to poll it and drive the built-in browser modal, and poll takeResult()
// to pick up a finished selection.
class FilePicker {
public:
    FilePicker();
    ~FilePicker();
    FilePicker(const FilePicker&) = delete;
    FilePicker& operator=(const FilePicker&) = delete;

    // Launches the native file dialog asynchronously. Falls back to
    // openBrowser() if neither zenity nor kdialog is installed.
    void openDialog();

    // Forces the built-in ImGui directory browser open.
    void openBrowser();

    // ImGui menu items listing the sample models; call inside an open
    // BeginMenu()/BeginPopup() block.
    void drawSamplesMenu();

    // Call every frame: polls background work and draws the built-in
    // browser modal (and any status popups) when active.
    void drawPopups();

    // Returns the path chosen since the last call, if any. "" means the
    // built-in test sphere (matches loadBody("")).
    std::optional<std::string> takeResult();

    // True while a native dialog or a sample decompression is in flight.
    bool busy() const;

    // Scans the OpenFOAM tutorial geometry folder for *.stl / *.stl.gz and
    // returns them alongside the built-in test sphere. Cached after the
    // first call; safe to call from drawSamplesMenu() every frame.
    const std::vector<SampleEntry>& listSamples();

private:
    struct BrowserEntry {
        std::string name;
        std::filesystem::path path;
        bool isDir = false;
    };

    void runNativeDialog(std::string exe, bool isZenity, std::string startDir);
    void pollNativeDialog();

    void requestSample(const SampleEntry& entry);
    void runDecompress(std::filesystem::path src, std::filesystem::path dest, std::filesystem::path destDir);
    void pollDecompress();

    void refreshBrowserEntries();
    void navigateTo(const std::filesystem::path& dir);

    // Native dialog thread state (written by the background thread, read
    // under dialogMutex_ from the UI thread).
    std::thread dialogThread_;
    std::atomic<bool> dialogRunning_{false};
    std::atomic<long long> dialogPid_{-1};
    std::mutex dialogMutex_;
    std::string dialogOutput_;
    bool dialogHasOutput_ = false;
    std::string lastDir_;

    // Sample decompression thread state.
    std::thread decompressThread_;
    std::atomic<bool> decompressRunning_{false};
    std::mutex decompressMutex_;
    std::string decompressResult_; // resulting path, or empty on failure
    std::string decompressError_;
    bool decompressHasResult_ = false;
    std::string decompressingDisplayName_; // for the "extracting..." label

    // UI-thread-only result outbox.
    std::optional<std::string> pendingResult_;
    std::string statusMessage_;

    // Samples (scanned once, lazily).
    std::vector<SampleEntry> samples_;
    bool samplesScanned_ = false;

    // Built-in browser modal state.
    bool browserOpen_ = false;
    bool browserNeedsOpen_ = false;
    std::filesystem::path browserDir_;
    std::vector<BrowserEntry> browserEntries_;
    char browserPathBuf_[1024] = {};
};

} // namespace app::ui

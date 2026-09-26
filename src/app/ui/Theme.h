#pragma once

#include <imgui.h>

namespace app::ui {

// Palette shared by the custom-drawn widgets.
namespace colour {
inline constexpr ImU32 kAccent = IM_COL32(61, 139, 253, 255);      // primary blue
inline constexpr ImU32 kAccentHover = IM_COL32(94, 160, 255, 255);
inline constexpr ImU32 kAccentActive = IM_COL32(40, 112, 220, 255);
inline constexpr ImU32 kPlay = IM_COL32(34, 197, 94, 255);         // play / go
inline constexpr ImU32 kPlayHover = IM_COL32(74, 222, 128, 255);
inline constexpr ImU32 kStop = IM_COL32(239, 68, 68, 255);         // cancel
inline constexpr ImU32 kStopHover = IM_COL32(248, 113, 113, 255);
inline constexpr ImU32 kText = IM_COL32(226, 232, 240, 255);
inline constexpr ImU32 kTextMuted = IM_COL32(139, 148, 160, 255);
inline constexpr ImU32 kWarning = IM_COL32(251, 191, 36, 255);
} // namespace colour

inline constexpr float kToolbarHeight = 56.0f;
inline constexpr float kStatusHeight = 28.0f;
inline constexpr float kLeftPanelWidth = 330.0f;
inline constexpr float kRightPanelWidth = 310.0f;
inline constexpr float kBaseFontSize = 15.0f;

// Load the UI font (bundled Roboto) and apply the dark slicer-style theme.
void applyTheme();

// Small caps section title with a hairline under it.
void sectionHeader(const char* label);

// Full-width coloured button with centred label. Returns true when clicked.
bool bigButton(const char* label, ImU32 colour, ImU32 hover, ImVec2 size);

// A row of mutually exclusive buttons (segmented control). Returns true when
// the selection changed.
bool segmented(const char* id, int* current, const char* const labels[], int count, float width);

// Muted helper text.
void hint(const char* fmt, ...);

} // namespace app::ui

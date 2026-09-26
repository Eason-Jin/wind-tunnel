cmake_minimum_required(VERSION 3.24)
project(windtunnel LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

find_package(Threads REQUIRED)
include(FetchContent)

# --- Third-party -------------------------------------------------------------

set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4)

FetchContent_Declare(glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.9b)

FetchContent_Declare(stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        2c980bb59875b0d32144a71867fbdebb2f77cd20)

FetchContent_MakeAvailable(glfw glm imgui stb)

add_library(glad STATIC external/glad/src/glad.c)
target_include_directories(glad PUBLIC external/glad/include)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC glfw)

add_library(stb INTERFACE)
target_include_directories(stb SYSTEM INTERFACE ${stb_SOURCE_DIR})

# --- Project modules ---------------------------------------------------------
# Each module globs its own directory, so new files inside a module need no
# CMake edits (re-run configure to pick them up).

set(WT_WARNINGS -Wall -Wextra)

file(GLOB_RECURSE WT_CORE_SRC CONFIGURE_DEPENDS src/core/*.cpp)
add_library(wt_core STATIC ${WT_CORE_SRC})
target_include_directories(wt_core PUBLIC src)
target_link_libraries(wt_core PUBLIC glm::glm)
target_compile_options(wt_core PRIVATE ${WT_WARNINGS})

file(GLOB_RECURSE WT_IO_SRC CONFIGURE_DEPENDS src/io/*.cpp)
add_library(wt_io STATIC ${WT_IO_SRC})
target_link_libraries(wt_io PUBLIC wt_core)
target_compile_options(wt_io PRIVATE ${WT_WARNINGS})

file(GLOB_RECURSE WT_SOLVERS_SRC CONFIGURE_DEPENDS src/solvers/*.cpp)
add_library(wt_solvers STATIC ${WT_SOLVERS_SRC})
target_link_libraries(wt_solvers PUBLIC wt_core wt_io Threads::Threads)
target_compile_options(wt_solvers PRIVATE ${WT_WARNINGS})

file(GLOB_RECURSE WT_RENDER_SRC CONFIGURE_DEPENDS src/render/*.cpp)
add_library(wt_render STATIC ${WT_RENDER_SRC})
target_link_libraries(wt_render PUBLIC wt_core glad imgui)
target_compile_definitions(wt_render PRIVATE WT_SHADER_DIR="${CMAKE_SOURCE_DIR}/shaders/")
target_compile_options(wt_render PRIVATE ${WT_WARNINGS})

file(GLOB_RECURSE WT_APP_SRC CONFIGURE_DEPENDS src/app/*.cpp)
add_executable(windtunnel ${WT_APP_SRC})
target_link_libraries(windtunnel PRIVATE wt_core wt_io wt_solvers wt_render glad glfw imgui stb)
target_compile_definitions(windtunnel PRIVATE WT_PROJECT_DIR="${CMAKE_SOURCE_DIR}")
target_compile_options(windtunnel PRIVATE ${WT_WARNINGS})

#pragma once

#include "render/gl/GlObjects.h"

#include <glm/glm.hpp>

namespace render {

enum class ColormapKind { Turbo = 0, Viridis, CoolWarm };
inline constexpr const char* kColormapNames[] = {"Turbo", "Viridis", "Cool-warm"};

// CPU evaluation of a colormap at t in [0,1].
glm::vec3 colormap(ColormapKind kind, float t);

// 256-texel 1D RGB texture (linear filtering, clamp-to-edge). Shaders sample
// it with texture(uColormap, t). Each pass owns its own (cheap) instance.
class ColormapTexture {
public:
    ColormapTexture() = default;
    explicit ColormapTexture(ColormapKind kind) { set(kind); }
    void set(ColormapKind kind);
    ColormapKind kind() const { return kind_; }
    void bind(GLuint unit) const { glBindTextureUnit(unit, tex_.id()); }

private:
    gl::Texture tex_;
    ColormapKind kind_ = ColormapKind::Turbo;
};

} // namespace render

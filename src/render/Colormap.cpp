#include "render/Colormap.h"

#include <algorithm>
#include <array>
#include <vector>

namespace render {

glm::vec3 colormap(ColormapKind kind, float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    switch (kind) {
    case ColormapKind::Turbo: {
        // Polynomial approximation of Google's Turbo (Mikhailov, 2019).
        const glm::vec4 kR(0.13572138f, 4.61539260f, -42.66032258f, 132.13108234f);
        const glm::vec4 kG(0.09140261f, 2.19418839f, 4.84296658f, -14.18503333f);
        const glm::vec4 kB(0.10667330f, 12.64194608f, -60.58204836f, 110.36276771f);
        const glm::vec2 kR2(-152.94239396f, 59.28637943f);
        const glm::vec2 kG2(4.27729857f, 2.82956604f);
        const glm::vec2 kB2(-89.90310912f, 27.34824973f);
        const glm::vec4 v4(1.0f, t, t * t, t * t * t);
        const glm::vec2 v2 = glm::vec2(v4.z, v4.w) * v4.z;
        return glm::clamp(glm::vec3(glm::dot(v4, kR) + glm::dot(v2, kR2), glm::dot(v4, kG) + glm::dot(v2, kG2),
                                    glm::dot(v4, kB) + glm::dot(v2, kB2)),
                          0.0f, 1.0f);
    }
    case ColormapKind::Viridis: {
        static const std::array<glm::vec3, 9> k = {{{0.267f, 0.005f, 0.329f}, {0.283f, 0.141f, 0.458f}, {0.254f, 0.265f, 0.530f},
                                                    {0.207f, 0.372f, 0.553f}, {0.164f, 0.471f, 0.558f}, {0.128f, 0.567f, 0.551f},
                                                    {0.135f, 0.659f, 0.518f}, {0.478f, 0.821f, 0.318f}, {0.993f, 0.906f, 0.144f}}};
        const float x = t * static_cast<float>(k.size() - 1);
        const std::size_t i = std::min(static_cast<std::size_t>(x), k.size() - 2);
        return glm::mix(k[i], k[i + 1], x - static_cast<float>(i));
    }
    case ColormapKind::CoolWarm: {
        const glm::vec3 cold(0.230f, 0.299f, 0.754f), mid(0.865f, 0.865f, 0.865f), warm(0.706f, 0.016f, 0.150f);
        return t < 0.5f ? glm::mix(cold, mid, t * 2.0f) : glm::mix(mid, warm, t * 2.0f - 1.0f);
    }
    }
    return glm::vec3(t);
}

void ColormapTexture::set(ColormapKind kind)
{
    kind_ = kind;
    constexpr int N = 256;
    std::vector<glm::vec3> texels(N);
    for (int i = 0; i < N; ++i)
        texels[i] = colormap(kind, static_cast<float>(i) / (N - 1));
    if (!tex_) {
        tex_ = gl::createTexture(GL_TEXTURE_1D);
        glTextureStorage1D(tex_.id(), 1, GL_RGB32F, N);
        glTextureParameteri(tex_.id(), GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTextureParameteri(tex_.id(), GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTextureParameteri(tex_.id(), GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    }
    glTextureSubImage1D(tex_.id(), 0, 0, N, GL_RGB, GL_FLOAT, texels.data());
}

} // namespace render

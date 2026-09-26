#include "render/passes/DomainPass.h"

#include <imgui.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace render {

namespace {
struct LineVertex {
    glm::vec3 pos;
    glm::vec4 colour;
};
} // namespace

DomainPass::DomainPass() : shader_(gl::Shader::fromFiles("lines.vert", "lines.frag")) {}

void DomainPass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    rebuild();
}

void DomainPass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    rebuild();
}

void DomainPass::rebuild()
{
    std::vector<LineVertex> v;
    core::Bounds box;
    if (scene_.field && !scene_.field->empty()) {
        box = scene_.field->bounds();
    } else if (scene_.body && !scene_.body->empty()) {
        const core::Bounds b = scene_.body->bounds();
        const float L = glm::length(b.size());
        box = {b.min - glm::vec3(L, L, 0.0f), b.max + glm::vec3(2.0f * L, L, L)};
        box.min.z = b.min.z;
    } else {
        vertexCount_ = 0;
        return;
    }

    const glm::vec4 edge(0.55f, 0.62f, 0.70f, 0.9f);
    const glm::vec3 lo = box.min, hi = box.max;
    const glm::vec3 c[8] = {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                            {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}};
    const int e[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& ed : e) {
        v.push_back({c[ed[0]], edge});
        v.push_back({c[ed[1]], edge});
    }

    if (showGrid_) {
        const glm::vec4 grid(0.45f, 0.50f, 0.56f, 0.35f);
        const glm::vec3 size = box.size();
        const float step = std::max(size.x, size.y) / 24.0f;
        for (float x = lo.x; x <= hi.x + 1e-4f; x += step) {
            v.push_back({{x, lo.y, lo.z}, grid});
            v.push_back({{x, hi.y, lo.z}, grid});
        }
        for (float y = lo.y; y <= hi.y + 1e-4f; y += step) {
            v.push_back({{lo.x, y, lo.z}, grid});
            v.push_back({{hi.x, y, lo.z}, grid});
        }
    }

    // Flow arrow at the inlet, pointing +x.
    const glm::vec4 arrow(0.35f, 0.75f, 1.0f, 1.0f);
    const float a = 0.12f * box.size().z + 1e-4f;
    const glm::vec3 mid(lo.x, 0.5f * (lo.y + hi.y), lo.z + 0.5f * (hi.z - lo.z));
    const glm::vec3 tip = mid + glm::vec3(2.0f * a, 0.0f, 0.0f);
    v.push_back({mid - glm::vec3(a, 0, 0), arrow});
    v.push_back({tip, arrow});
    v.push_back({tip, arrow});
    v.push_back({tip + glm::vec3(-0.6f * a, 0, 0.4f * a), arrow});
    v.push_back({tip, arrow});
    v.push_back({tip + glm::vec3(-0.6f * a, 0, -0.4f * a), arrow});

    vertexCount_ = static_cast<int>(v.size());
    vbo_ = gl::createBuffer();
    glNamedBufferStorage(vbo_.id(), static_cast<GLsizeiptr>(v.size() * sizeof(LineVertex)), v.data(), 0);
    vao_ = gl::createVertexArray();
    glVertexArrayVertexBuffer(vao_.id(), 0, vbo_.id(), 0, sizeof(LineVertex));
    glEnableVertexArrayAttrib(vao_.id(), 0);
    glVertexArrayAttribFormat(vao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(LineVertex, pos));
    glVertexArrayAttribBinding(vao_.id(), 0, 0);
    glEnableVertexArrayAttrib(vao_.id(), 1);
    glVertexArrayAttribFormat(vao_.id(), 1, 4, GL_FLOAT, GL_FALSE, offsetof(LineVertex, colour));
    glVertexArrayAttribBinding(vao_.id(), 1, 0);
}

void DomainPass::draw(const FrameContext& frame)
{
    if (vertexCount_ == 0)
        return;
    shader_.set("uViewProj", frame.proj * frame.view);
    shader_.use();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(vao_.id());
    glDrawArrays(GL_LINES, 0, vertexCount_);
    glDisable(GL_BLEND);
}

void DomainPass::drawUi()
{
    if (ImGui::Checkbox("Floor grid", &showGrid_))
        rebuild();
}

} // namespace render

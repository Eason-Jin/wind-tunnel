#include "render/passes/ModelPass.h"

#include <imgui.h>

#include <algorithm>
#include <cstddef>

namespace render {

namespace {
struct PosNorm {
    glm::vec3 pos;
    glm::vec3 normal;
};
} // namespace

ModelPass::ModelPass()
    : shader_(gl::Shader::fromFiles("model.vert", "model.frag")),
      wireShader_(gl::Shader::fromFiles("model.vert", "model_wire.frag")),
      colormap_(ColormapKind::Turbo)
{
}

ModelPass::~ModelPass() = default;

void ModelPass::onBodyChanged(const SceneRefs& scene)
{
    scene_ = scene;
    rebuildGeometry();
}

void ModelPass::onFieldChanged(const SceneRefs& scene)
{
    scene_ = scene;
    recomputeScalar();
    uploadScalar();
}

void ModelPass::update(const FrameContext&) {}

void ModelPass::rebuildGeometry()
{
    vertexCount_ = 0;
    if (!scene_.body || scene_.body->empty()) {
        cp_.clear();
        return;
    }

    const core::SurfaceMesh& body = *scene_.body;
    vertexCount_ = static_cast<int>(body.positions.size());

    std::vector<PosNorm> verts(static_cast<std::size_t>(vertexCount_));
    for (std::size_t i = 0; i < verts.size(); ++i) {
        verts[i].pos = body.positions[i];
        verts[i].normal = body.normals[i];
    }

    posNormVbo_ = gl::createBuffer();
    glNamedBufferStorage(posNormVbo_.id(), static_cast<GLsizeiptr>(verts.size() * sizeof(PosNorm)), verts.data(), 0);

    recomputeScalar();

    vao_ = gl::createVertexArray();
    glVertexArrayVertexBuffer(vao_.id(), 0, posNormVbo_.id(), 0, sizeof(PosNorm));
    glEnableVertexArrayAttrib(vao_.id(), 0);
    glVertexArrayAttribFormat(vao_.id(), 0, 3, GL_FLOAT, GL_FALSE, offsetof(PosNorm, pos));
    glVertexArrayAttribBinding(vao_.id(), 0, 0);
    glEnableVertexArrayAttrib(vao_.id(), 1);
    glVertexArrayAttribFormat(vao_.id(), 1, 3, GL_FLOAT, GL_FALSE, offsetof(PosNorm, normal));
    glVertexArrayAttribBinding(vao_.id(), 1, 0);

    uploadScalar();
}

void ModelPass::recomputeScalar()
{
    if (!scene_.body || scene_.body->empty()) {
        cp_.clear();
        return;
    }
    const core::SurfaceMesh& body = *scene_.body;
    if (cp_.size() != body.positions.size())
        cp_.assign(body.positions.size(), 0.0f);

    if (!scene_.field || scene_.field->empty()) {
        std::fill(cp_.begin(), cp_.end(), 0.0f);
        return;
    }

    const core::FlowField& field = *scene_.field;
    const float cell = std::max(field.spacing.x, 1e-6f);
    const float baseOffset = 0.75f * cell;
    const float maxOffset = 3.0f * cell;
    const float u = field.freestreamSpeed;
    const float denom = 0.5f * u * u;

    for (std::size_t i = 0; i < body.positions.size(); ++i) {
        const glm::vec3& p = body.positions[i];
        glm::vec3 n = body.normals[i];
        const float len = glm::length(n);
        n = len > 1e-8f ? n / len : glm::vec3(1.0f, 0.0f, 0.0f);

        glm::vec3 samplePos = p + n * baseOffset;
        bool found = false;
        for (int side = 0; side < 2 && !found; ++side) {
            const glm::vec3 dir = side == 0 ? n : -n;
            for (float off = baseOffset; off <= maxOffset + 1e-4f; off += cell) {
                const glm::vec3 sp = p + dir * off;
                samplePos = sp;
                if (!field.isSolid(sp)) {
                    found = true;
                    break;
                }
            }
        }

        const float pressure = field.samplePressure(samplePos);
        cp_[i] = denom > 1e-8f ? pressure / denom : 0.0f;
    }
}

void ModelPass::uploadScalar()
{
    scalarVbo_ = gl::createBuffer();
    if (!cp_.empty()) {
        glNamedBufferStorage(scalarVbo_.id(), static_cast<GLsizeiptr>(cp_.size() * sizeof(float)), cp_.data(), 0);
    } else {
        const float zero = 0.0f;
        glNamedBufferStorage(scalarVbo_.id(), sizeof(float), &zero, 0);
    }
    if (vao_) {
        glVertexArrayVertexBuffer(vao_.id(), 1, scalarVbo_.id(), 0, sizeof(float));
        glEnableVertexArrayAttrib(vao_.id(), 2);
        glVertexArrayAttribFormat(vao_.id(), 2, 1, GL_FLOAT, GL_FALSE, 0);
        glVertexArrayAttribBinding(vao_.id(), 2, 1);
    }
}

void ModelPass::computeAutoRange()
{
    if (cp_.empty())
        return;
    float mn = cp_.front();
    float mx = cp_.front();
    for (const float v : cp_) {
        mn = std::min(mn, v);
        mx = std::max(mx, v);
    }
    if (mx - mn < 1e-4f)
        mx = mn + 1e-4f;
    cpMin_ = mn;
    cpMax_ = mx;
}

void ModelPass::draw(const FrameContext& frame)
{
    if (vertexCount_ == 0 || !vao_)
        return;

    const bool fieldValid = scene_.field && !scene_.field->empty();
    const ColorMode mode = (colorMode_ == ColorMode::SurfaceCp && fieldValid) ? ColorMode::SurfaceCp : ColorMode::Clay;

    const bool useBlend = opacity_ < 0.999f;
    if (useBlend) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDepthMask(GL_FALSE);
    }

    if (wireframe_) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
    }

    if (clipPlane_)
        glEnable(GL_CLIP_DISTANCE0);
    const glm::vec4 clip = clipPlane_.value_or(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
    shader_.set("uClipPlane", clip);
    wireShader_.set("uClipPlane", clip);

    shader_.use();
    shader_.set("uViewProj", frame.proj * frame.view);
    shader_.set("uEye", frame.eye);
    shader_.set("uClayColor", clayColor_);
    shader_.set("uColorMode", mode == ColorMode::SurfaceCp ? 1 : 0);
    shader_.set("uScalarMin", cpMin_);
    shader_.set("uScalarMax", cpMax_);
    shader_.set("uOpacity", opacity_);
    if (mode == ColorMode::SurfaceCp) {
        colormap_.bind(kColormapUnit);
        shader_.set("uColormap", static_cast<int>(kColormapUnit));
    }

    glBindVertexArray(vao_.id());
    glDrawArrays(GL_TRIANGLES, 0, vertexCount_);

    if (wireframe_) {
        glDisable(GL_POLYGON_OFFSET_FILL);
        wireShader_.use();
        wireShader_.set("uViewProj", frame.proj * frame.view);
        wireShader_.set("uWireColor", glm::vec4(0.05f, 0.06f, 0.08f, opacity_));
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (useBlend) {
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
    }
    glDisable(GL_CLIP_DISTANCE0);
}

void ModelPass::drawUi()
{
    const bool fieldValid = scene_.field && !scene_.field->empty();

    int mode = static_cast<int>(colorMode_);
    ImGui::BeginDisabled(!fieldValid);
    if (ImGui::Combo("Colour", &mode, "Clay\0Surface Cp\0"))
        colorMode_ = static_cast<ColorMode>(mode);
    ImGui::EndDisabled();

    if (colorMode_ == ColorMode::Clay || !fieldValid)
        ImGui::ColorEdit3("Clay colour", &clayColor_.x);

    if (colorMode_ == ColorMode::SurfaceCp && fieldValid) {
        int cmIndex = colormapIndex_;
        if (ImGui::Combo("Colormap", &cmIndex, kColormapNames, IM_ARRAYSIZE(kColormapNames))) {
            colormapIndex_ = cmIndex;
            colormap_.set(static_cast<ColormapKind>(colormapIndex_));
        }
        ImGui::DragFloat("Cp min", &cpMin_, 0.01f);
        ImGui::SameLine();
        ImGui::DragFloat("Cp max", &cpMax_, 0.01f);
        if (ImGui::Button("Auto range"))
            computeAutoRange();

        // Horizontal colour bar with min/max labels.
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const float barW = std::max(ImGui::GetContentRegionAvail().x, 32.0f);
        const float barH = 16.0f;
        constexpr int kSteps = 64;
        for (int i = 0; i < kSteps; ++i) {
            const float t0 = static_cast<float>(i) / kSteps;
            const float t1 = static_cast<float>(i + 1) / kSteps;
            const glm::vec3 c = colormap(static_cast<ColormapKind>(colormapIndex_), t0);
            const ImU32 col = IM_COL32(static_cast<int>(c.r * 255.0f), static_cast<int>(c.g * 255.0f),
                                       static_cast<int>(c.b * 255.0f), 255);
            draw->AddRectFilled(ImVec2(p.x + t0 * barW, p.y), ImVec2(p.x + t1 * barW, p.y + barH), col);
        }
        ImGui::Dummy(ImVec2(barW, barH));
        ImGui::Text("%.2f", static_cast<double>(cpMin_));
        ImGui::SameLine(std::max(barW - 40.0f, 0.0f));
        ImGui::Text("%.2f", static_cast<double>(cpMax_));
    }

    ImGui::Checkbox("Wireframe overlay", &wireframe_);
    ImGui::SliderFloat("Opacity", &opacity_, 0.0f, 1.0f);
}

} // namespace render

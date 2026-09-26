#pragma once

#include <glad/glad.h>

#include <utility>

namespace render::gl {

// Move-only RAII owner of one OpenGL object name. Destroy with the matching
// glDelete* when the owner goes out of scope. Must be destroyed while the GL
// context is still current.
template <void (*Delete)(GLuint)>
class Handle {
public:
    Handle() = default;
    explicit Handle(GLuint id) : id_(id) {}
    ~Handle() { reset(); }

    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    Handle(Handle&& other) noexcept : id_(std::exchange(other.id_, 0)) {}
    Handle& operator=(Handle&& other) noexcept
    {
        if (this != &other) {
            reset();
            id_ = std::exchange(other.id_, 0);
        }
        return *this;
    }

    GLuint id() const { return id_; }
    explicit operator bool() const { return id_ != 0; }
    void reset()
    {
        if (id_)
            Delete(id_);
        id_ = 0;
    }

private:
    GLuint id_ = 0;
};

namespace detail {
inline void deleteBuffer(GLuint id) { glDeleteBuffers(1, &id); }
inline void deleteVertexArray(GLuint id) { glDeleteVertexArrays(1, &id); }
inline void deleteTexture(GLuint id) { glDeleteTextures(1, &id); }
inline void deleteFramebuffer(GLuint id) { glDeleteFramebuffers(1, &id); }
inline void deleteRenderbuffer(GLuint id) { glDeleteRenderbuffers(1, &id); }
} // namespace detail

using Buffer = Handle<detail::deleteBuffer>;
using VertexArray = Handle<detail::deleteVertexArray>;
using Texture = Handle<detail::deleteTexture>;
using Framebuffer = Handle<detail::deleteFramebuffer>;
using Renderbuffer = Handle<detail::deleteRenderbuffer>;

// Creation helpers (GL 4.5 direct state access).
inline Buffer createBuffer()
{
    GLuint id = 0;
    glCreateBuffers(1, &id);
    return Buffer(id);
}
inline VertexArray createVertexArray()
{
    GLuint id = 0;
    glCreateVertexArrays(1, &id);
    return VertexArray(id);
}
inline Texture createTexture(GLenum target)
{
    GLuint id = 0;
    glCreateTextures(target, 1, &id);
    return Texture(id);
}

} // namespace render::gl

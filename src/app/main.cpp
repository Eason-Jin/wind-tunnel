#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <iostream>

int main() {
    glm::vec3 v(1.0f, 2.0f, 3.0f);
    std::cout << "GLFW " << glfwGetVersionString() << '\n'
              << "GLM vec3.y = " << v.y << '\n';
    return 0;
}
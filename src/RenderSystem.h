#pragma once

#include <entt/entt.hpp>
#include <raylib.h>

namespace RenderSystem {
    void init    (int width, int height, const char* title);
    void render  (entt::registry& reg, Camera3D& camera, const char* sceneName);
    void shutdown();
}

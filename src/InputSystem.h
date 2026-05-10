#pragma once

#include <entt/entt.hpp>
#include <raylib.h>

class PhysicsWorld;

namespace InputSystem {
    // Creates the persistent kinematic drag body — call once before the game loop.
    void init  (PhysicsWorld& world, entt::registry& reg);

    // Per-frame update: camera orbit + mouse pick/drag.
    void update(PhysicsWorld& world, entt::registry& reg, Camera3D& camera);
}

#pragma once

#include <entt/entt.hpp>

class PhysicsWorld;

struct IScene {
    virtual ~IScene() = default;
    virtual void        init    (PhysicsWorld& world, entt::registry& reg) = 0;
    virtual void        update  (PhysicsWorld& world, entt::registry& reg, float dt) = 0;
    virtual const char* name    () const = 0;
};

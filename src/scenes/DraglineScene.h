#pragma once
#include "IScene.h"

class DraglineScene : public IScene {
public:
    void        init  (PhysicsWorld& world, entt::registry& reg) override;
    void        update(PhysicsWorld& world, entt::registry& reg, float dt) override {}
    const char* name  () const override { return "Dragline Rope (press 2 for joints)"; }
};

#pragma once
#include "IScene.h"

class JointTestScene : public IScene {
public:
    void        init  (PhysicsWorld& world, entt::registry& reg) override;
    void        update(PhysicsWorld& world, entt::registry& reg, float dt) override {}
    const char* name  () const override { return "Joint Test: pendulum + spring (press 1 for dragline)"; }
};

#pragma once
#include "IScene.h"
#include <PxPhysicsAPI.h>

class CraneScene : public IScene {
public:
    void        init  (PhysicsWorld& world, entt::registry& reg) override;
    void        update(PhysicsWorld& world, entt::registry& reg, float dt) override;
    const char* name  () const override;

private:
    entt::entity mBase   = entt::null;
    entt::entity mBoom   = entt::null;
    entt::entity mBucket = entt::null;

    physx::PxDistanceJoint* mHoistJoint = nullptr;
    physx::PxDistanceJoint* mDragJoint  = nullptr;

    float mBoomAngle   = 0.0f;
    float mHoistLength = 8.0f;
    float mDragLength  = 7.0f;
};

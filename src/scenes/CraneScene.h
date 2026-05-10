#pragma once
#include "IScene.h"
#include <PxPhysicsAPI.h>

class CraneScene : public IScene {
public:
    void        init  (PhysicsWorld& world, entt::registry& reg) override;
    void        update(PhysicsWorld& world, entt::registry& reg, float dt) override;
    const char* name  () const override;

private:
    entt::entity mBoom   = entt::null;
    entt::entity mBucket = entt::null;

    physx::PxDistanceJoint* mHoistJoint = nullptr;  // boom-tip → bucket (vertical rope)
    physx::PxDistanceJoint* mDragJoint  = nullptr;  // body-base → bucket (pull rope)

    float mBoomAngle   = 0.0f;   // yaw in radians
    float mHoistLength = 4.5f;   // max hoist rope length  (W/S to change)
    float mDragLength  = 9.0f;   // max drag rope length   (Q/E to change)

    static constexpr float BOOM_HALF     = 5.0f;   // half-extent of boom along its axis
    static constexpr float PIVOT_HEIGHT  = 3.2f;   // world Y of the boom pivot
};

#include "CraneScene.h"
#include "PhysicsWorld.h"
#include "Components.h"

#include <raylib.h>
#include <PxPhysicsAPI.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {
    constexpr float BOOM_HALF    = 4.5f;
    constexpr float PIVOT_HEIGHT = 3.0f;   // world Y of the boom pivot (= top of base)
    constexpr float BASE_HALF_Y  = 1.5f;   // base body half-height

    // Boom rotation: fixed 45° pitch up, then yaw around world Y.
    physx::PxQuat boomRotation(float yaw) {
        physx::PxQuat pitch(physx::PxPi / 4.0f, physx::PxVec3(0, 0, 1)); // tilt +X toward +Y
        physx::PxQuat yawQ (yaw,                 physx::PxVec3(0, 1, 0));
        return yawQ * pitch;  // PhysX quat mul: pitch first, then yaw
    }

    // Boom center = pivot + half-length along the boom's forward direction.
    // Deriving center from rotation (not from trig formulas) keeps pivot consistent.
    physx::PxVec3 boomCenter(float yaw) {
        physx::PxVec3 dir = boomRotation(yaw).rotate(physx::PxVec3(1, 0, 0));
        return physx::PxVec3(0, PIVOT_HEIGHT, 0) + dir * BOOM_HALF;
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────

void CraneScene::init(PhysicsWorld& world, entt::registry& reg) {
    mBoomAngle   = 0.0f;
    mHoistLength = 8.0f;
    mDragLength  = 7.0f;
    mHoistJoint  = nullptr;
    mDragJoint   = nullptr;

    // ── Ground ────────────────────────────────────────────────────────────────
    {
        auto e = world.createStaticBox(reg,
            physx::PxVec3(0, -0.5f, 0), physx::PxVec3(30, 0.5f, 30));
        reg.emplace<RenderComponent>(e, Color{40, 80, 40, 255});
    }

    // ── Machine body — kinematic so it rotates with the boom ─────────────────
    // Base center sits at y = BASE_HALF_Y; pivot is at y = PIVOT_HEIGHT (top of base).
    mBase = world.createKinematicBox(reg,
        physx::PxTransform(physx::PxVec3(0, BASE_HALF_Y, 0)),
        physx::PxVec3(0.9f, BASE_HALF_Y, 0.9f));
    reg.emplace<RenderComponent>(mBase, Color{110, 80, 50, 255});

    // ── Boom — kinematic, 45° pitch, no collision ─────────────────────────────
    mBoom = world.createKinematicBox(reg,
        physx::PxTransform(boomCenter(0.0f), boomRotation(0.0f)),
        physx::PxVec3(BOOM_HALF, 0.18f, 0.28f));
    reg.emplace<RenderComponent>(mBoom, Color{200, 160, 40, 255});

    // ── Bucket ────────────────────────────────────────────────────────────────
    // Start near the ground in front of the base.
    mBucket = world.createDynamicSphere(reg,
        physx::PxVec3(4.5f, 0.8f, 0.0f), 0.4f, 10.0f);
    reg.emplace<RenderComponent>(mBucket, Color{60, 120, 200, 255});
    reg.emplace<DragTargetComponent>(mBucket);

    // ── Hoist rope: boom TIP → bucket ─────────────────────────────────────────
    // frameA is at the tip end of the boom in boom-local space (+BOOM_HALF along local X).
    {
        physx::PxTransform frameA(physx::PxVec3(BOOM_HALF, 0, 0));
        physx::PxTransform frameB(physx::PxIdentity);
        auto jc = world.createDistanceJoint(reg,
            mBoom, mBucket, 0.2f, mHoistLength,
            4000.0f, 400.0f, frameA, frameB);
        mHoistJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);
        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);
    }

    // ── Drag (pull) rope: base FRONT → bucket ─────────────────────────────────
    // frameA at the front face of the base (local +X direction) so the rope
    // attachment rotates with the machine body when slewing.
    {
        physx::PxTransform frameA(physx::PxVec3(0.9f, 0.0f, 0.0f));
        physx::PxTransform frameB(physx::PxIdentity);
        auto jc = world.createDistanceJoint(reg,
            mBase, mBucket, 0.2f, mDragLength,
            4000.0f, 400.0f, frameA, frameB);
        mDragJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);
        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);
    }
}

// ── Update ────────────────────────────────────────────────────────────────────

void CraneScene::update(PhysicsWorld& world, entt::registry& reg, float dt) {
    constexpr float SLEW_SPEED  = 1.0f;
    constexpr float HOIST_SPEED = 2.0f;
    constexpr float DRAG_SPEED  = 2.0f;

    if (IsKeyDown(KEY_A)) mBoomAngle -= SLEW_SPEED * dt;
    if (IsKeyDown(KEY_D)) mBoomAngle += SLEW_SPEED * dt;
    if (IsKeyDown(KEY_W)) mHoistLength = std::max(1.5f, mHoistLength - HOIST_SPEED * dt);
    if (IsKeyDown(KEY_S)) mHoistLength = std::min(12.0f, mHoistLength + HOIST_SPEED * dt);
    if (IsKeyDown(KEY_Q)) mDragLength  = std::max(1.5f, mDragLength  - DRAG_SPEED  * dt);
    if (IsKeyDown(KEY_E)) mDragLength  = std::min(18.0f, mDragLength  + DRAG_SPEED  * dt);

    // ── Boom: yaw + fixed 45° pitch ───────────────────────────────────────────
    world.setKinematicPose(mBoom, reg,
        physx::PxTransform(boomCenter(mBoomAngle), boomRotation(mBoomAngle)));

    // ── Base: same yaw, no pitch — rotates in place ───────────────────────────
    physx::PxQuat baseYaw(mBoomAngle, physx::PxVec3(0, 1, 0));
    world.setKinematicPose(mBase, reg,
        physx::PxTransform(physx::PxVec3(0, BASE_HALF_Y, 0), baseYaw));

    if (mHoistJoint) mHoistJoint->setMaxDistance(mHoistLength);
    if (mDragJoint)  mDragJoint->setMaxDistance(mDragLength);
}

// ── Name ──────────────────────────────────────────────────────────────────────

const char* CraneScene::name() const {
    static char buf[160];
    snprintf(buf, sizeof(buf),
        "Crane  A/D:slew  W/S:hoist %.1fm  Q/E:drag %.1fm",
        mHoistLength, mDragLength);
    return buf;
}

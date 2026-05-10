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
    constexpr float PIVOT_HEIGHT = 3.0f;
    constexpr float BASE_HALF_Y  = 1.5f;

    physx::PxQuat boomRotation(float yaw) {
        physx::PxQuat pitch(physx::PxPi / 4.0f, physx::PxVec3(0, 0, 1));
        physx::PxQuat yawQ (yaw,                 physx::PxVec3(0, 1, 0));
        return yawQ * pitch;
    }

    physx::PxVec3 boomCenter(float yaw) {
        physx::PxVec3 dir = boomRotation(yaw).rotate(physx::PxVec3(1, 0, 0));
        return physx::PxVec3(0, PIVOT_HEIGHT, 0) + dir * BOOM_HALF;
    }
}

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

    // ── Base (kinematic, rotates with boom) ───────────────────────────────────
    mBase = world.createKinematicBox(reg,
        physx::PxTransform(physx::PxVec3(0, BASE_HALF_Y, 0)),
        physx::PxVec3(0.9f, BASE_HALF_Y, 0.9f));
    reg.emplace<RenderComponent>(mBase, Color{110, 80, 50, 255});

    // ── Boom (kinematic, 45° pitch) ───────────────────────────────────────────
    mBoom = world.createKinematicBox(reg,
        physx::PxTransform(boomCenter(0.0f), boomRotation(0.0f)),
        physx::PxVec3(BOOM_HALF, 0.18f, 0.28f));
    reg.emplace<RenderComponent>(mBoom, Color{200, 160, 40, 255});

    // ── Bucket ────────────────────────────────────────────────────────────────
    mBucket = world.createDynamicSphere(reg,
        physx::PxVec3(4.5f, 0.8f, 0.0f), 0.4f, 50.0f);
    reg.emplace<RenderComponent>(mBucket, Color{60, 120, 200, 255});
    reg.emplace<DragTargetComponent>(mBucket);

    auto& bucketRb = reg.get<RigidBodyComponent>(mBucket);
    auto* bucketActor = static_cast<physx::PxRigidDynamic*>(bucketRb.actor);
    bucketActor->setSolverIterationCounts(32, 4);

    // ── Anti-swing D6 joint: locks the boom's LOCAL Z axis ────────────────────
    // The bucket is free to move in the boom's XY plane (in/out and up/down)
    // but CANNOT move laterally (local Z = sideways relative to the boom).
    // This is physically correct: real draglines keep the bucket in the
    // boom's vertical plane through chain geometry — no sideways swing.
    // As the boom slews (A/D), the locked plane rotates with it, so the
    // anti-swing constraint automatically follows the new direction.
    /*{
        auto& boomRb = reg.get<RigidBodyComponent>(mBoom);
        physx::PxD6Joint* j = physx::PxD6JointCreate(*world.getPhysics(),
            boomRb.actor,   physx::PxTransform(physx::PxIdentity),
            bucketActor,    physx::PxTransform(physx::PxIdentity));

        j->setMotion(physx::PxD6Axis::eX,      physx::PxD6Motion::eFREE);
        j->setMotion(physx::PxD6Axis::eY,      physx::PxD6Motion::eFREE);
        j->setMotion(physx::PxD6Axis::eZ,      physx::PxD6Motion::eLOCKED); // no lateral swing
        j->setMotion(physx::PxD6Axis::eTWIST,  physx::PxD6Motion::eFREE);
        j->setMotion(physx::PxD6Axis::eSWING1, physx::PxD6Motion::eFREE);
        j->setMotion(physx::PxD6Axis::eSWING2, physx::PxD6Motion::eFREE);

        // Register in the registry so clearActors() releases it automatically.
        // bodyA/B are null so drawJoints() skips rendering this invisible constraint.
        JointComponent jc{};
        jc.type  = JointComponent::Type::D6;
        jc.joint = j;
        jc.bodyA = entt::null;
        jc.bodyB = entt::null;
        reg.emplace<JointComponent>(reg.create(), jc);
    }*/

    // ── Hoist: boom-tip → bucket (hard distance, max only) ───────────────────
    {
        physx::PxTransform frameA(physx::PxVec3(BOOM_HALF, 0, 0));
        physx::PxTransform frameB(physx::PxIdentity);
        auto jc = world.createDistanceJoint(reg,
            mBoom, mBucket, 0.2f, mHoistLength,
            0.0f, 0.0f, frameA, frameB);
        mHoistJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);
        reg.emplace<JointComponent>(reg.create(), jc);
    }

    // ── Drag: base-front → bucket (hard distance, max only) ──────────────────
    {
        physx::PxTransform frameA(physx::PxVec3(0.9f, 0, 0));
        physx::PxTransform frameB(physx::PxIdentity);
        auto jc = world.createDistanceJoint(reg,
            mBase, mBucket, 0.2f, mDragLength,
            0.0f, 0.0f, frameA, frameB);
        mDragJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);
        reg.emplace<JointComponent>(reg.create(), jc);
    }
}

void CraneScene::update(PhysicsWorld& world, entt::registry& reg, float dt) {
    constexpr float SLEW_SPEED  = 1.0f;
    constexpr float HOIST_SPEED = 2.0f;
    constexpr float DRAG_SPEED  = 2.0f;

    if (IsKeyDown(KEY_A)) mBoomAngle -= SLEW_SPEED * dt;
    if (IsKeyDown(KEY_D)) mBoomAngle += SLEW_SPEED * dt;

    float prevHoist = mHoistLength;
    float prevDrag  = mDragLength;

    if (IsKeyDown(KEY_W)) mHoistLength = std::max(1.0f,  mHoistLength - HOIST_SPEED * dt);
    if (IsKeyDown(KEY_S)) mHoistLength = std::min(12.0f, mHoistLength + HOIST_SPEED * dt);
    if (IsKeyDown(KEY_Q)) mDragLength  = std::max(1.0f,  mDragLength  - DRAG_SPEED  * dt);
    if (IsKeyDown(KEY_E)) mDragLength  = std::min(18.0f, mDragLength  + DRAG_SPEED  * dt);

    world.setKinematicPose(mBoom, reg,
        physx::PxTransform(boomCenter(mBoomAngle), boomRotation(mBoomAngle)));

    physx::PxQuat baseYaw(mBoomAngle, physx::PxVec3(0, 1, 0));
    world.setKinematicPose(mBase, reg,
        physx::PxTransform(physx::PxVec3(0, BASE_HALF_Y, 0), baseYaw));

    if (mHoistLength != prevHoist && mHoistJoint)
        mHoistJoint->setMaxDistance(mHoistLength);
    if (mDragLength != prevDrag && mDragJoint)
        mDragJoint->setMaxDistance(mDragLength);
}

const char* CraneScene::name() const {
    static char buf[160];
    snprintf(buf, sizeof(buf),
        "Crane  A/D:slew  W/S:hoist %.1fm  Q/E:drag %.1fm",
        mHoistLength, mDragLength);
    return buf;
}

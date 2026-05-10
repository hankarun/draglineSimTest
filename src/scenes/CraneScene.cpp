#include "CraneScene.h"
#include "PhysicsWorld.h"
#include "Components.h"

#include <raylib.h>
#include <PxPhysicsAPI.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

// ── Init ──────────────────────────────────────────────────────────────────────

void CraneScene::init(PhysicsWorld& world, entt::registry& reg) {
    mBoomAngle   = 0.0f;
    mHoistLength = 4.5f;
    mDragLength  = 9.0f;
    mHoistJoint  = nullptr;
    mDragJoint   = nullptr;

    // ── Ground ────────────────────────────────────────────────────────────────
    {
        auto e = world.createStaticBox(reg,
            physx::PxVec3(0, -0.5f, 0), physx::PxVec3(30, 0.5f, 30));
        reg.emplace<RenderComponent>(e, Color{40, 80, 40, 255});
    }

    // ── Machine body (static base) ────────────────────────────────────────────
    // Acts as the fixed crane body sitting on the ground.
    entt::entity base = world.createStaticBox(reg,
        physx::PxVec3(0, 1.6f, 0), physx::PxVec3(1.0f, 1.6f, 1.0f));
    reg.emplace<RenderComponent>(base, Color{110, 80, 50, 255});  // brown

    // ── Boom (kinematic box, no collision) ────────────────────────────────────
    // The boom pivots at (0, PIVOT_HEIGHT, 0).
    // Its center sits at (BOOM_HALF, PIVOT_HEIGHT, 0) when angle = 0,
    // so the base-end of the boom is at the pivot and the tip is at 2*BOOM_HALF.
    {
        physx::PxTransform boomPose(
            physx::PxVec3(BOOM_HALF, PIVOT_HEIGHT, 0.0f),
            physx::PxQuat(physx::PxIdentity));
        mBoom = world.createKinematicBox(reg, boomPose,
            physx::PxVec3(BOOM_HALF, 0.18f, 0.3f));
        reg.emplace<RenderComponent>(mBoom, Color{200, 160, 40, 255});  // golden yellow
    }

    // ── Bucket (dynamic box, draggable) ──────────────────────────────────────
    // Starts near the ground roughly under the boom tip.
    mBucket = world.createDynamicSphere(reg,
        physx::PxVec3(8.0f, 0.8f, 0.0f), 0.4f, 8.0f);
    reg.emplace<RenderComponent>(mBucket, Color{60, 120, 200, 255});  // steel blue
    reg.emplace<DragTargetComponent>(mBucket);

    // ── Hoist rope: boom TIP → bucket ────────────────────────────────────────
    // frameA is at the boom tip in the boom's local frame (+BOOM_HALF along local X).
    {
        physx::PxTransform frameA(physx::PxVec3(BOOM_HALF, 0.0f, 0.0f));
        physx::PxTransform frameB(physx::PxIdentity);

        auto jc = world.createDistanceJoint(reg,
            mBoom, mBucket,
            0.2f,           // min distance (stops bucket hitting boom tip)
            mHoistLength,
            4000.0f,        // stiff spring — feels like an inextensible rope
            400.0f,
            frameA, frameB);

        mHoistJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);
    }

    // ── Drag rope: base center → bucket ──────────────────────────────────────
    // The pull rope connects the machine body to the bucket.
    {
        physx::PxTransform frameA(physx::PxIdentity);
        physx::PxTransform frameB(physx::PxIdentity);

        auto jc = world.createDistanceJoint(reg,
            base, mBucket,
            0.2f,
            mDragLength,
            4000.0f,
            400.0f,
            frameA, frameB);

        mDragJoint = static_cast<physx::PxDistanceJoint*>(jc.joint);

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);
    }
}

// ── Update ────────────────────────────────────────────────────────────────────

void CraneScene::update(PhysicsWorld& world, entt::registry& reg, float dt) {
    constexpr float SLEW_SPEED  = 1.0f;   // rad/s
    constexpr float HOIST_SPEED = 2.0f;   // m/s
    constexpr float DRAG_SPEED  = 2.0f;   // m/s

    // A/D — slew boom left / right
    if (IsKeyDown(KEY_A)) mBoomAngle -= SLEW_SPEED * dt;
    if (IsKeyDown(KEY_D)) mBoomAngle += SLEW_SPEED * dt;

    // W/S — raise / lower bucket (shorten / lengthen hoist rope)
    if (IsKeyDown(KEY_W)) mHoistLength = std::max(1.5f, mHoistLength - HOIST_SPEED * dt);
    if (IsKeyDown(KEY_S)) mHoistLength = std::min(10.0f, mHoistLength + HOIST_SPEED * dt);

    // Q/E — pull bucket in / let it out (shorten / lengthen drag rope)
    if (IsKeyDown(KEY_Q)) mDragLength = std::max(1.5f, mDragLength - DRAG_SPEED * dt);
    if (IsKeyDown(KEY_E)) mDragLength = std::min(18.0f, mDragLength + DRAG_SPEED * dt);

    // ── Update boom kinematic pose ────────────────────────────────────────────
    // Rotate center around the pivot point (0, PIVOT_HEIGHT, 0).
    physx::PxVec3 boomCenter(
        BOOM_HALF * cosf(mBoomAngle),
        PIVOT_HEIGHT,
        BOOM_HALF * sinf(mBoomAngle));
    physx::PxQuat boomRot(mBoomAngle, physx::PxVec3(0, 1, 0));
    world.setKinematicPose(mBoom, reg, physx::PxTransform(boomCenter, boomRot));

    // ── Update rope lengths ───────────────────────────────────────────────────
    if (mHoistJoint) mHoistJoint->setMaxDistance(mHoistLength);
    if (mDragJoint)  mDragJoint->setMaxDistance(mDragLength);
}

// ── Name (shown in HUD, includes live rope lengths) ──────────────────────────

const char* CraneScene::name() const {
    static char buf[160];
    snprintf(buf, sizeof(buf),
        "Crane  A/D: slew  W/S: hoist %.1fm  Q/E: drag %.1fm",
        mHoistLength, mDragLength);
    return buf;
}

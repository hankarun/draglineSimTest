#include "JointTestScene.h"
#include "PhysicsWorld.h"
#include "Components.h"

#include <raylib.h>
#include <PxPhysicsAPI.h>

void JointTestScene::init(PhysicsWorld& world, entt::registry& reg) {
    // ── Ground plane ──────────────────────────────────────────────────────────
    entt::entity ground = world.createStaticBox(reg,
        physx::PxVec3(0.0f, -0.5f, 0.0f),
        physx::PxVec3(20.0f, 0.5f, 20.0f));
    reg.emplace<RenderComponent>(ground, Color{ 40, 80, 40, 255 });

    // ── Pendulum (revolute joint) ─────────────────────────────────────────────
    // Hinge rotates around the joint frame's X axis.
    // Rotating the frame 90° around Z makes X point in world Y, so the pendulum
    // swings left-right in the XY plane when given a Z-axis velocity kick.
    // Actually we want it to swing in X, so: default orientation rotates around X,
    // meaning the bob swings in the YZ plane. We give it a +Z velocity kick.
    {
        entt::entity pivot = world.createStaticBox(reg,
            physx::PxVec3(-4.0f, 6.5f, 0.0f),
            physx::PxVec3(0.2f, 0.2f, 0.2f));
        reg.emplace<RenderComponent>(pivot, Color{ 80, 80, 80, 255 });

        entt::entity bob = world.createDynamicSphere(reg,
            physx::PxVec3(-4.0f, 3.0f, 0.0f), 0.3f, 2.0f);
        reg.emplace<RenderComponent>(bob, RED);
        reg.emplace<DragTargetComponent>(bob);

        // frameA: joint is at pivot center (no offset, hinge around default X axis).
        // frameB: joint is 3.5 units above the bob center (at the pivot height).
        physx::PxTransform frameA(physx::PxIdentity);
        physx::PxTransform frameB(physx::PxVec3(0.0f, 3.5f, 0.0f));

        JointComponent jc = world.createRevoluteJoint(reg,
            pivot, bob, frameA, frameB,
            true,                               // enable angular limit
            -physx::PxPi * 0.8f,               // ≈ -144°
             physx::PxPi * 0.8f);              // ≈  144°

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);

        // Kick the bob in Z so it starts swinging (hinge is around X → swings in YZ).
        auto* bobActor = static_cast<physx::PxRigidDynamic*>(
            reg.get<RigidBodyComponent>(bob).actor);
        bobActor->setLinearVelocity(physx::PxVec3(0.0f, 0.0f, 3.5f));
    }

    // ── Bouncy spring (distance joint with spring enabled) ───────────────────
    // A fixed anchor suspends a heavy sphere via a spring — it bounces up and down.
    {
        entt::entity anchor = world.createStaticBox(reg,
            physx::PxVec3(4.0f, 7.0f, 0.0f),
            physx::PxVec3(0.2f, 0.2f, 0.2f));
        reg.emplace<RenderComponent>(anchor, Color{ 80, 80, 80, 255 });

        entt::entity weight = world.createDynamicSphere(reg,
            physx::PxVec3(4.0f, 3.5f, 0.0f), 0.35f, 3.0f);
        reg.emplace<RenderComponent>(weight, SKYBLUE);
        reg.emplace<DragTargetComponent>(weight);

        // Distance joint: min=1 (compressed), max=5 (stretched), spring stiffness=120.
        JointComponent jc = world.createDistanceJoint(reg,
            anchor, weight,
            1.0f,    // min distance
            5.0f,    // max distance
            120.0f,  // spring stiffness — pulls back toward rest length
            15.0f);  // damping

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);

        // Push the weight down so the spring starts compressed.
        auto* weightActor = static_cast<physx::PxRigidDynamic*>(
            reg.get<RigidBodyComponent>(weight).actor);
        weightActor->setLinearVelocity(physx::PxVec3(0.0f, -4.0f, 0.0f));
    }

    // ── D6 spring joint (two free bodies connected by spring drives) ──────────
    // Two spheres that attract each other with a spring — demonstrates D6 joint.
    {
        entt::entity ballA = world.createDynamicSphere(reg,
            physx::PxVec3(-1.0f, 4.0f, -3.0f), 0.3f, 1.5f);
        entt::entity ballB = world.createDynamicSphere(reg,
            physx::PxVec3( 1.0f, 4.0f, -3.0f), 0.3f, 1.5f);
        reg.emplace<RenderComponent>(ballA, LIME);
        reg.emplace<RenderComponent>(ballB, VIOLET);
        reg.emplace<DragTargetComponent>(ballA);
        reg.emplace<DragTargetComponent>(ballB);

        JointComponent jc = world.createSpringJoint(reg,
            ballA, ballB,
            150.0f,   // stiffness
            18.0f,    // damping
            2.0f);    // rest length

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);

        // Kick both balls in opposite directions so the spring oscillates.
        auto* aActor = static_cast<physx::PxRigidDynamic*>(
            reg.get<RigidBodyComponent>(ballA).actor);
        auto* bActor = static_cast<physx::PxRigidDynamic*>(
            reg.get<RigidBodyComponent>(ballB).actor);
        aActor->setLinearVelocity(physx::PxVec3(-3.0f, 2.0f, 0.0f));
        bActor->setLinearVelocity(physx::PxVec3( 3.0f, 2.0f, 0.0f));
    }
}

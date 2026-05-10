#pragma once

#include <PxPhysicsAPI.h>
#include <entt/entt.hpp>
#include "Components.h"

class PhysicsWorld {
public:
    void init();
    void shutdown();

    // Clamps dt to 1/30 s, then simulate() + fetchResults(blocking).
    void step(float dt);

    // ── Body factories ────────────────────────────────────────────────────────
    // Each factory creates an entity, attaches RigidBodyComponent + ShapeComponent,
    // and registers the actor with the PxScene.

    entt::entity createDynamicSphere(entt::registry& reg,
                                     physx::PxVec3 pos,
                                     float radius,
                                     float density = 1.0f);

    entt::entity createStaticBox(entt::registry& reg,
                                 physx::PxVec3 pos,
                                 physx::PxVec3 halfExtents);

    // Invisible kinematic sphere used as the drag anchor.
    entt::entity createKinematicSphere(entt::registry& reg,
                                       physx::PxVec3 pos);

    // Move kinematic body to a world position (used by the drag system).
    void setKinematicTarget(entt::entity e, entt::registry& reg,
                            physx::PxVec3 worldPos);
    // Set full pose (position + rotation) of a kinematic body.
    void setKinematicPose(entt::entity e, entt::registry& reg,
                          physx::PxTransform pose);

    // Kinematic box — shape collision disabled so it acts as a ghost.
    entt::entity createKinematicBox(entt::registry& reg,
                                    physx::PxTransform pose,
                                    physx::PxVec3 halfExtents);

    // ── Joint factories ───────────────────────────────────────────────────────
    // Return a filled JointComponent; caller emplaces it on a registry entity.

    // frameA / frameB define the joint attachment in each body's local space.
    // The p (translation) component of each frame is stored in JointComponent for rendering.
    JointComponent createDistanceJoint(entt::registry& reg,
                                       entt::entity a,
                                       entt::entity b,
                                       float minDist,
                                       float maxDist,
                                       float stiffness = 0.0f,
                                       float damping   = 0.0f,
                                       physx::PxTransform frameA = physx::PxTransform(physx::PxIdentity),
                                       physx::PxTransform frameB = physx::PxTransform(physx::PxIdentity));

    JointComponent createRevoluteJoint(entt::registry& reg,
                                       entt::entity a,
                                       entt::entity b,
                                       physx::PxTransform localFrameA,
                                       physx::PxTransform localFrameB,
                                       bool  enableLimit = false,
                                       float lower       = -physx::PxPi,
                                       float upper       =  physx::PxPi);

    // D6 joint with spring drives — use this for spring-like connections.
    JointComponent createSpringJoint(entt::registry& reg,
                                     entt::entity a,
                                     entt::entity b,
                                     float stiffness,
                                     float damping,
                                     float restLength);

    // High-stiffness D6 spring joint for mouse drag (soft pull).
    physx::PxD6Joint* createDragJoint(physx::PxRigidActor* kinematic,
                                      physx::PxRigidActor* target,
                                      physx::PxVec3        grabLocalPos);

    void releaseJoint(physx::PxJoint* joint);

    // Release all joints + actors in the registry and remove them from the scene.
    // Call this before registry.clear() when switching scenes.
    void clearActors(entt::registry& reg);

    physx::PxPhysics*  getPhysics()  const { return mPhysics;  }
    physx::PxScene*    getScene()    const { return mScene;    }
    physx::PxMaterial* getMaterial() const { return mMaterial; }

private:
    physx::PxDefaultAllocator      mAllocator;
    physx::PxDefaultErrorCallback  mErrorCallback;
    physx::PxFoundation*           mFoundation  = nullptr;
    physx::PxPhysics*              mPhysics     = nullptr;
    physx::PxDefaultCpuDispatcher* mDispatcher  = nullptr;
    physx::PxScene*                mScene       = nullptr;
    physx::PxMaterial*             mMaterial    = nullptr;
    physx::PxPvd*                  mPvd         = nullptr;
};

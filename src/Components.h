#pragma once

#include <PxPhysicsAPI.h>
#include <raylib.h>
#include <entt/entt.hpp>

// ── Physics ───────────────────────────────────────────────────────────────────

struct RigidBodyComponent {
    physx::PxRigidActor* actor       = nullptr;
    bool                 isKinematic = false;
};

struct ShapeComponent {
    enum class Type { Sphere, Box };
    Type  type    = Type::Sphere;
    float radius  = 0.5f;
    float halfX   = 0.5f;
    float halfY   = 0.5f;
    float halfZ   = 0.5f;
};

struct JointComponent {
    enum class Type { Distance, Revolute, D6 };
    Type              type;
    physx::PxJoint*   joint         = nullptr;
    entt::entity      bodyA         = entt::null;
    entt::entity      bodyB         = entt::null;
    float             stiffness     = 0.0f;
    float             damping       = 0.0f;
    float             restLength    = 1.0f;
    // Attachment points in each body's local frame — used for accurate rope rendering.
    physx::PxVec3     localOffsetA  = { 0, 0, 0 };
    physx::PxVec3     localOffsetB  = { 0, 0, 0 };
};

// ── Rendering ─────────────────────────────────────────────────────────────────

struct RenderComponent {
    Color color   = WHITE;
    bool  visible = true;
};

// ── Interaction ───────────────────────────────────────────────────────────────

struct DragTargetComponent {};  // tag: entity can be picked with the mouse

struct DragStateComponent {
    entt::entity      pickedEntity = entt::null;
    physx::PxD6Joint* dragJoint    = nullptr;
    float             grabDepth    = 0.0f;
};

// ── Scene-specific ────────────────────────────────────────────────────────────

struct ChainLinkComponent {
    int  linkIndex = 0;
    bool isBucket  = false;
};

#include "InputSystem.h"
#include "PhysicsWorld.h"
#include "Components.h"
#include "Utils.h"

#include <PxPhysicsAPI.h>
#include <raymath.h>
#include <cfloat>

namespace InputSystem {

// ── Persistent state ──────────────────────────────────────────────────────────

static entt::entity gDragBodyEntity = entt::null;
static bool         gIsDragging     = false;
static float        gGrabDepth      = 0.0f;

// ── Camera orbit ──────────────────────────────────────────────────────────────

static void updateCamera(Camera3D& camera) {
    // Orbit: right-mouse drag rotates the camera around its target.
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
        Vector2 delta = GetMouseDelta();
        const float sensitivity = 0.005f;

        Vector3 arm = Vector3Subtract(camera.position, camera.target);
        float   len = Vector3Length(arm);

        // Yaw around world Y
        if (delta.x != 0.0f) {
            Matrix rotY = MatrixRotateY(-delta.x * sensitivity);
            arm = Vector3Transform(arm, rotY);
        }

        // Pitch around the camera's right axis, clamped to avoid gimbal lock.
        if (delta.y != 0.0f) {
            Vector3 right   = Vector3Normalize(Vector3CrossProduct(arm, camera.up));
            Vector3 armNorm = Vector3Normalize(arm);
            float currentPitch = asinf(Clamp(armNorm.y, -1.0f, 1.0f));
            float newPitch     = Clamp(currentPitch + delta.y * sensitivity, -1.4f, 1.4f);
            float pitchDelta   = newPitch - currentPitch;
            Matrix rotR = MatrixRotate(right, pitchDelta);
            arm = Vector3Transform(arm, rotR);
        }

        arm = Vector3Scale(Vector3Normalize(arm), len);
        camera.position = Vector3Add(camera.target, arm);
    }

    // Zoom: scroll dolly along the view direction.
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        Vector3 dir  = Vector3Subtract(camera.target, camera.position);
        float   dist = Vector3Length(dir);
        float   step = wheel * dist * 0.1f;
        if (dist - step > 0.5f) {  // don't zoom past the target
            camera.position = Vector3Add(camera.position,
                Vector3Scale(Vector3Normalize(dir), step));
        }
    }
}

// ── Pick ──────────────────────────────────────────────────────────────────────

static entt::entity pickEntity(entt::registry& reg, Ray ray) {
    entt::entity closest = entt::null;
    float        minDist = FLT_MAX;

    auto view = reg.view<RigidBodyComponent, ShapeComponent, DragTargetComponent>();
    for (auto e : view) {
        auto& rb    = view.get<RigidBodyComponent>(e);
        auto& shape = view.get<ShapeComponent>(e);
        if (!rb.actor) continue;

        Vector3 center = PxToRl(rb.actor->getGlobalPose().p);

        RayCollision col = {};
        if (shape.type == ShapeComponent::Type::Sphere) {
            col = GetRayCollisionSphere(ray, center, shape.radius);
        } else {
            Vector3 half = { shape.halfX, shape.halfY, shape.halfZ };
            BoundingBox bb = {
                Vector3Subtract(center, half),
                Vector3Add(center, half)
            };
            col = GetRayCollisionBox(ray, bb);
        }

        if (col.hit && col.distance < minDist) {
            minDist = col.distance;
            closest = e;
        }
    }
    return closest;
}

// ── Drag ──────────────────────────────────────────────────────────────────────

static void beginDrag(PhysicsWorld& world, entt::registry& reg,
                      entt::entity picked, Vector3 grabWorldPos) {
    // Move the kinematic drag body to the grab point.
    world.setKinematicTarget(gDragBodyEntity, reg, RlToPx(grabWorldPos));

    // Compute the grab point in the picked body's local frame.
    auto& rbPicked = reg.get<RigidBodyComponent>(picked);
    physx::PxTransform invPose = rbPicked.actor->getGlobalPose().getInverse();
    physx::PxVec3 grabLocal    = invPose.transform(RlToPx(grabWorldPos));

    auto& rbDrag = reg.get<RigidBodyComponent>(gDragBodyEntity);
    physx::PxD6Joint* joint = world.createDragJoint(rbDrag.actor, rbPicked.actor, grabLocal);

    reg.emplace_or_replace<DragStateComponent>(gDragBodyEntity,
        picked, joint, gGrabDepth);

    gIsDragging = true;
}

static void updateDrag(PhysicsWorld& world, entt::registry& reg, Camera3D& camera) {
    Ray     ray      = GetMouseRay(GetMousePosition(), camera);
    Vector3 newWorld = Vector3Add(ray.position,
                           Vector3Scale(ray.direction, gGrabDepth));
    world.setKinematicTarget(gDragBodyEntity, reg, RlToPx(newWorld));
}

static void endDrag(PhysicsWorld& world, entt::registry& reg) {
    if (reg.all_of<DragStateComponent>(gDragBodyEntity)) {
        auto& ds = reg.get<DragStateComponent>(gDragBodyEntity);
        world.releaseJoint(ds.dragJoint);
        reg.remove<DragStateComponent>(gDragBodyEntity);
    }
    // Park the drag body far below the scene.
    world.setKinematicTarget(gDragBodyEntity, reg, physx::PxVec3(0.0f, -200.0f, 0.0f));
    gIsDragging = false;
}

// ── Public API ────────────────────────────────────────────────────────────────

void init(PhysicsWorld& world, entt::registry& reg) {
    gDragBodyEntity = world.createKinematicSphere(reg, physx::PxVec3(0.0f, -200.0f, 0.0f));
    gIsDragging     = false;
    gGrabDepth      = 0.0f;
}

void update(PhysicsWorld& world, entt::registry& reg, Camera3D& camera) {
    updateCamera(camera);

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Ray ray = GetMouseRay(GetMousePosition(), camera);
        entt::entity hit = pickEntity(reg, ray);
        if (hit != entt::null) {
            auto& rb  = reg.get<RigidBodyComponent>(hit);
            Vector3 bodyPos = PxToRl(rb.actor->getGlobalPose().p);
            gGrabDepth      = Vector3Length(Vector3Subtract(bodyPos, camera.position));
            Vector3 grabPt  = Vector3Add(ray.position,
                                Vector3Scale(ray.direction, gGrabDepth));
            beginDrag(world, reg, hit, grabPt);
        }
    }

    if (gIsDragging) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            updateDrag(world, reg, camera);
        } else {
            endDrag(world, reg);
        }
    }
}

} // namespace InputSystem

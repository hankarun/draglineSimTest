#include "RenderSystem.h"
#include "Components.h"
#include "Utils.h"

#include <PxPhysicsAPI.h>
#include <raylib.h>
#include <rlgl.h>
#include <raymath.h>
#include <cstdio>

namespace RenderSystem {

void init(int width, int height, const char* title) {
    InitWindow(width, height, title);
    SetTargetFPS(60);
}

void shutdown() {
    CloseWindow();
}

// ── Internal draw helpers ─────────────────────────────────────────────────────

static void drawBodies(entt::registry& reg) {
    auto view = reg.view<RigidBodyComponent, ShapeComponent, RenderComponent>();
    for (auto e : view) {
        auto& rb    = view.get<RigidBodyComponent>(e);
        auto& shape = view.get<ShapeComponent>(e);
        auto& rc    = view.get<RenderComponent>(e);
        if (!rc.visible || !rb.actor) continue;

        physx::PxTransform t = rb.actor->getGlobalPose();
        Vector3 pos = PxToRl(t.p);

        if (shape.type == ShapeComponent::Type::Sphere) {
            DrawSphere(pos, shape.radius, rc.color);
            DrawSphereWires(pos, shape.radius, 8, 8, Fade(WHITE, 0.25f));
        } else {
            // Extract axis-angle from quaternion for rlgl rotation.
            physx::PxVec3 axis;
            physx::PxReal angle;
            t.q.toRadiansAndUnitAxis(angle, axis);
            Vector3 rlAxis = PxToRl(axis);

            // rlPushMatrix stacks with DrawCube's own internal rlPushMatrix,
            // so the cube is rendered at pos with the given orientation.
            rlPushMatrix();
            rlTranslatef(pos.x, pos.y, pos.z);
            rlRotatef(angle * RAD2DEG, rlAxis.x, rlAxis.y, rlAxis.z);
            DrawCube(     {0,0,0}, shape.halfX*2, shape.halfY*2, shape.halfZ*2, rc.color);
            DrawCubeWires({0,0,0}, shape.halfX*2, shape.halfY*2, shape.halfZ*2, Fade(WHITE, 0.35f));
            rlPopMatrix();
        }
    }
}

static void drawJoints(entt::registry& reg) {
    auto view = reg.view<JointComponent>();
    for (auto e : view) {
        auto& jc = view.get<JointComponent>(e);
        if (!reg.valid(jc.bodyA) || !reg.valid(jc.bodyB)) continue;

        auto* rbA = reg.try_get<RigidBodyComponent>(jc.bodyA);
        auto* rbB = reg.try_get<RigidBodyComponent>(jc.bodyB);
        if (!rbA || !rbB || !rbA->actor || !rbB->actor) continue;

        // World-space attachment points using the per-joint local offsets.
        Vector3 posA = PxToRl(rbA->actor->getGlobalPose().transform(jc.localOffsetA));
        Vector3 posB = PxToRl(rbB->actor->getGlobalPose().transform(jc.localOffsetB));

        Color lineColor;
        switch (jc.type) {
            case JointComponent::Type::Distance: lineColor = BLUE;   break;
            case JointComponent::Type::Revolute: lineColor = GREEN;  break;
            case JointComponent::Type::D6:       lineColor = YELLOW; break;
            default:                             lineColor = WHITE;  break;
        }
        DrawLine3D(posA, posB, lineColor);
        DrawSphere(posA, 0.06f, lineColor);
        DrawSphere(posB, 0.06f, lineColor);
    }
}

static void drawDragLine(entt::registry& reg) {
    auto view = reg.view<DragStateComponent, RigidBodyComponent>();
    for (auto e : view) {
        auto& ds = view.get<DragStateComponent>(e);
        auto& rb = view.get<RigidBodyComponent>(e);
        if (!reg.valid(ds.pickedEntity)) continue;

        auto* pickedRb = reg.try_get<RigidBodyComponent>(ds.pickedEntity);
        if (!pickedRb || !pickedRb->actor) continue;

        Vector3 dragPos   = PxToRl(rb.actor->getGlobalPose().p);
        Vector3 pickedPos = PxToRl(pickedRb->actor->getGlobalPose().p);
        DrawLine3D(dragPos, pickedPos, RED);
        DrawSphere(dragPos, 0.1f, RED);
    }
}

static void drawHUD(const char* sceneName) {
    char buf[256];
    snprintf(buf, sizeof(buf), "FPS: %d | %s", GetFPS(), sceneName);
    DrawText(buf, 10, 10, 18, RAYWHITE);
    DrawText("LMB: drag  |  RMB+drag: orbit  |  Scroll: zoom  |  1/2/3: switch scene",
             10, 34, 16, LIGHTGRAY);
    DrawText("Joint colors:  BLUE = distance/rope  |  GREEN = revolute  |  YELLOW = D6/spring",
             10, 56, 16, LIGHTGRAY);
}

// ── Public entry point ────────────────────────────────────────────────────────

void render(entt::registry& reg, Camera3D& camera, const char* sceneName) {
    BeginDrawing();
    ClearBackground(Color{ 20, 20, 30, 255 });

    BeginMode3D(camera);
        DrawGrid(40, 1.0f);
        drawBodies(reg);
        drawJoints(reg);
        drawDragLine(reg);
    EndMode3D();

    drawHUD(sceneName);
    EndDrawing();
}

} // namespace RenderSystem

#include <raylib.h>
#include <entt/entt.hpp>
#include <memory>
#include <vector>

#include "PhysicsWorld.h"
#include "RenderSystem.h"
#include "InputSystem.h"
#include "scenes/IScene.h"
#include "scenes/DraglineScene.h"
#include "scenes/JointTestScene.h"
#include "scenes/CraneScene.h"

static void loadScene(int idx,
                      std::vector<std::unique_ptr<IScene>>& scenes,
                      PhysicsWorld& world,
                      entt::registry& reg) {
    // Release all PhysX actors/joints, then wipe the registry.
    world.clearActors(reg);
    reg.clear();

    // Re-create the drag body that InputSystem owns.
    InputSystem::init(world, reg);

    scenes[idx]->init(world, reg);
}

int main() {
    // ── Window ────────────────────────────────────────────────────────────────
    RenderSystem::init(1280, 720, "PhysX 5 Simulator  |  Raylib + EnTT");

    // ── Camera ────────────────────────────────────────────────────────────────
    Camera3D camera      = {};
    camera.position      = { 0.0f, 12.0f, 22.0f };
    camera.target        = { 0.0f,  3.0f,  0.0f };
    camera.up            = { 0.0f,  1.0f,  0.0f };
    camera.fovy          = 45.0f;
    camera.projection    = CAMERA_PERSPECTIVE;

    // ── Physics ───────────────────────────────────────────────────────────────
    PhysicsWorld world;
    world.init();

    // ── ECS registry ─────────────────────────────────────────────────────────
    entt::registry registry;

    // ── Scenes ────────────────────────────────────────────────────────────────
    std::vector<std::unique_ptr<IScene>> scenes;
    scenes.push_back(std::make_unique<DraglineScene>());
    scenes.push_back(std::make_unique<JointTestScene>());
    scenes.push_back(std::make_unique<CraneScene>());

    int currentScene = 0;
    InputSystem::init(world, registry);
    scenes[currentScene]->init(world, registry);

    // ── Game loop ─────────────────────────────────────────────────────────────
    while (!WindowShouldClose()) {
        // Scene switching with keys 1 and 2.
        for (int i = 0; i < static_cast<int>(scenes.size()); ++i) {
            if (IsKeyPressed(KEY_ONE + i) && i != currentScene) {
                currentScene = i;
                loadScene(currentScene, scenes, world, registry);
            }
        }

        // Scene update (crane controls, etc.) runs before physics.
        scenes[currentScene]->update(world, registry, GetFrameTime());

        // Input must run before the physics step so the drag body is already
        // at its new position when the solver runs.
        InputSystem::update(world, registry, camera);

        world.step(GetFrameTime());

        RenderSystem::render(registry, camera, scenes[currentScene]->name());
    }

    // ── Cleanup ───────────────────────────────────────────────────────────────
    world.clearActors(registry);
    registry.clear();
    world.shutdown();
    RenderSystem::shutdown();

    return 0;
}

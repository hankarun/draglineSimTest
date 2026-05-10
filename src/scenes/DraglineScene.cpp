#include "DraglineScene.h"
#include "PhysicsWorld.h"
#include "Components.h"

#include <raylib.h>

void DraglineScene::init(PhysicsWorld& world, entt::registry& reg) {
    // ── Fixed anchor ─────────────────────────────────────────────────────────
    // Small dark box clamped to the ceiling — not draggable.
    entt::entity anchor = world.createStaticBox(reg,
        physx::PxVec3(0.0f, 9.0f, 0.0f),
        physx::PxVec3(0.25f, 0.25f, 0.25f));
    reg.emplace<RenderComponent>(anchor, Color{ 80, 80, 80, 255 });

    // ── Chain links ───────────────────────────────────────────────────────────
    // 9 links; the last one is the "bucket" (heavier, larger, orange).
    const int   NUM_LINKS    = 9;
    const float LINK_SPACING = 0.85f;  // natural rest length between links
    const float LINK_RADIUS  = 0.15f;
    const float BUCKET_RADIUS = 0.38f;

    entt::entity prev = anchor;

    for (int i = 0; i < NUM_LINKS; ++i) {
        bool  isBucket = (i == NUM_LINKS - 1);
        float y        = 9.0f - (i + 1) * LINK_SPACING;

        entt::entity link = world.createDynamicSphere(reg,
            physx::PxVec3(0.0f, y, 0.0f),
            isBucket ? BUCKET_RADIUS : LINK_RADIUS,
            isBucket ? 4.0f : 1.0f);

        reg.emplace<RenderComponent>(link,
            isBucket ? ORANGE : Color{ 200, 200, 200, 255 });
        reg.emplace<DragTargetComponent>(link);
        reg.emplace<ChainLinkComponent>(link, i, isBucket);

        // Near-inextensible link: min ≈ max so the distance barely changes.
        // High stiffness + heavy damping suppress bounce at the limit.
        JointComponent jc = world.createDistanceJoint(reg,
            prev, link,
            LINK_SPACING * 0.97f,  // min (very close to max — no compression)
            LINK_SPACING,          // max (hard rope length)
            3000.0f,               // stiff spring kills any slack/stretch
            500.0f);               // heavy damping kills oscillation

        auto je = reg.create();
        reg.emplace<JointComponent>(je, jc);

        prev = link;
    }

    // ── Ground plane ──────────────────────────────────────────────────────────
    entt::entity ground = world.createStaticBox(reg,
        physx::PxVec3(0.0f, -0.5f, 0.0f),
        physx::PxVec3(20.0f, 0.5f, 20.0f));
    reg.emplace<RenderComponent>(ground, Color{ 40, 80, 40, 255 });
}

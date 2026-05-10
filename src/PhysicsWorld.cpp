#include "PhysicsWorld.h"

using namespace physx;

void PhysicsWorld::init() {
    mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, mAllocator, mErrorCallback);

    // Try to connect to PhysX Visual Debugger on localhost:5425.
    // The simulator runs fine if PVD is not open.
    mPvd = PxCreatePvd(*mFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
    mPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation,
                               PxTolerancesScale(), true, mPvd);

    // PxInitExtensions is required before using any joint creation functions.
    PxInitExtensions(*mPhysics, mPvd);

    mDispatcher = PxDefaultCpuDispatcherCreate(2);

    PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
    sceneDesc.gravity       = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc.cpuDispatcher = mDispatcher;
    sceneDesc.filterShader  = PxDefaultSimulationFilterShader;
    // TGS solver eliminates the compliance ("rubber band") that PGS exhibits on
    // hard distance constraints. It is strictly better for chain/rope simulations.
    sceneDesc.solverType    = PxSolverType::eTGS;
    mScene = mPhysics->createScene(sceneDesc);

    // Transmit full scene data to PVD so the debugger shows constraints and contacts.
    PxPvdSceneClient* pvdClient = mScene->getScenePvdClient();
    if (pvdClient) {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS,    true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES,true);
    }

    mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0.2f);
}

void PhysicsWorld::shutdown() {
    if (mMaterial)   { mMaterial->release();    mMaterial   = nullptr; }
    if (mScene)      { mScene->release();        mScene      = nullptr; }
    if (mDispatcher) { mDispatcher->release();   mDispatcher = nullptr; }
    PxCloseExtensions();
    if (mPhysics)    { mPhysics->release();      mPhysics    = nullptr; }
    if (mPvd) {
        PxPvdTransport* t = mPvd->getTransport();
        mPvd->release();
        mPvd = nullptr;
        if (t) t->release();
    }
    if (mFoundation) { mFoundation->release();   mFoundation = nullptr; }
}

void PhysicsWorld::step(float dt) {
    if (dt > 1.0f / 30.0f) dt = 1.0f / 30.0f;
    mScene->simulate(dt);
    mScene->fetchResults(true);
}

// ── Body factories ────────────────────────────────────────────────────────────

entt::entity PhysicsWorld::createDynamicSphere(entt::registry& reg,
                                               PxVec3 pos,
                                               float radius,
                                               float density) {
    PxRigidDynamic* body = mPhysics->createRigidDynamic(PxTransform(pos));
    PxRigidActorExt::createExclusiveShape(*body, PxSphereGeometry(radius), *mMaterial);
    PxRigidBodyExt::updateMassAndInertia(*body, density);
    mScene->addActor(*body);

    auto e = reg.create();
    reg.emplace<RigidBodyComponent>(e, (PxRigidActor*)body, false);
    reg.emplace<ShapeComponent>(e, ShapeComponent::Type::Sphere, radius);
    return e;
}

entt::entity PhysicsWorld::createStaticBox(entt::registry& reg,
                                           PxVec3 pos,
                                           PxVec3 halfExtents) {
    PxRigidStatic* body = mPhysics->createRigidStatic(PxTransform(pos));
    PxRigidActorExt::createExclusiveShape(*body,
        PxBoxGeometry(halfExtents.x, halfExtents.y, halfExtents.z),
        *mMaterial);
    mScene->addActor(*body);

    auto e = reg.create();
    reg.emplace<RigidBodyComponent>(e, (PxRigidActor*)body, false);
    reg.emplace<ShapeComponent>(e, ShapeComponent::Type::Box,
                                0.0f,
                                halfExtents.x, halfExtents.y, halfExtents.z);
    return e;
}

entt::entity PhysicsWorld::createKinematicSphere(entt::registry& reg, PxVec3 pos) {
    PxRigidDynamic* body = mPhysics->createRigidDynamic(PxTransform(pos));
    body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

    // Tiny shape required by PhysX — disable all collision and query flags so it
    // is completely invisible to the simulation.
    PxShape* shape = PxRigidActorExt::createExclusiveShape(
        *body, PxSphereGeometry(0.05f), *mMaterial);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, false);

    mScene->addActor(*body);

    auto e = reg.create();
    reg.emplace<RigidBodyComponent>(e, (PxRigidActor*)body, true);
    return e;
}

void PhysicsWorld::setKinematicTarget(entt::entity e, entt::registry& reg,
                                      PxVec3 worldPos) {
    auto* dynamic = static_cast<PxRigidDynamic*>(reg.get<RigidBodyComponent>(e).actor);
    dynamic->setKinematicTarget(PxTransform(worldPos));
}

void PhysicsWorld::setKinematicPose(entt::entity e, entt::registry& reg,
                                    PxTransform pose) {
    auto* dynamic = static_cast<PxRigidDynamic*>(reg.get<RigidBodyComponent>(e).actor);
    dynamic->setKinematicTarget(pose);
}

entt::entity PhysicsWorld::createKinematicBox(entt::registry& reg,
                                              PxTransform pose,
                                              PxVec3 halfExtents) {
    PxRigidDynamic* body = mPhysics->createRigidDynamic(pose);
    body->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

    // Disable collision + scene query: the boom is a ghost for physics purposes
    // (joints attach to the actor, not the shape, so joints still work).
    PxShape* shape = PxRigidActorExt::createExclusiveShape(
        *body,
        PxBoxGeometry(halfExtents.x, halfExtents.y, halfExtents.z),
        *mMaterial);
    shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
    shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE,  false);

    mScene->addActor(*body);

    auto e = reg.create();
    reg.emplace<RigidBodyComponent>(e, (PxRigidActor*)body, true);
    reg.emplace<ShapeComponent>(e, ShapeComponent::Type::Box, 0.0f,
                                halfExtents.x, halfExtents.y, halfExtents.z);
    return e;
}

// ── Joint factories ───────────────────────────────────────────────────────────

JointComponent PhysicsWorld::createDistanceJoint(entt::registry& reg,
                                                 entt::entity a,
                                                 entt::entity b,
                                                 float minDist,
                                                 float maxDist,
                                                 float stiffness,
                                                 float damping,
                                                 PxTransform frameA,
                                                 PxTransform frameB) {
    auto* actorA = reg.get<RigidBodyComponent>(a).actor;
    auto* actorB = reg.get<RigidBodyComponent>(b).actor;

    PxDistanceJoint* joint = PxDistanceJointCreate(*mPhysics,
        actorA, frameA,
        actorB, frameB);

    joint->setMinDistance(minDist);
    joint->setMaxDistance(maxDist);
    joint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, true);
    joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);

    if (stiffness > 0.0f) {
        joint->setStiffness(stiffness);
        joint->setDamping(damping);
        joint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, true);
    }

    JointComponent jc;
    jc.type         = JointComponent::Type::Distance;
    jc.joint        = joint;
    jc.bodyA        = a;
    jc.bodyB        = b;
    jc.stiffness    = stiffness;
    jc.damping      = damping;
    jc.restLength   = (minDist + maxDist) * 0.5f;
    jc.localOffsetA = frameA.p;
    jc.localOffsetB = frameB.p;
    return jc;
}

JointComponent PhysicsWorld::createRevoluteJoint(entt::registry& reg,
                                                 entt::entity a,
                                                 entt::entity b,
                                                 PxTransform localFrameA,
                                                 PxTransform localFrameB,
                                                 bool  enableLimit,
                                                 float lower,
                                                 float upper) {
    auto* actorA = reg.get<RigidBodyComponent>(a).actor;
    auto* actorB = reg.get<RigidBodyComponent>(b).actor;

    PxRevoluteJoint* joint = PxRevoluteJointCreate(*mPhysics,
        actorA, localFrameA,
        actorB, localFrameB);

    if (enableLimit) {
        // PhysX 5: third arg is PxSpring(stiffness, damping), not a contact distance.
        // PxSpring(0,0) = hard limit.
        PxJointAngularLimitPair limits(lower, upper, PxSpring(0.0f, 0.0f));
        joint->setLimit(limits);
        joint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, true);
    }

    JointComponent jc;
    jc.type  = JointComponent::Type::Revolute;
    jc.joint = joint;
    jc.bodyA = a;
    jc.bodyB = b;
    return jc;
}

JointComponent PhysicsWorld::createSpringJoint(entt::registry& reg,
                                               entt::entity a,
                                               entt::entity b,
                                               float stiffness,
                                               float damping,
                                               float restLength) {
    auto* actorA = reg.get<RigidBodyComponent>(a).actor;
    auto* actorB = reg.get<RigidBodyComponent>(b).actor;

    PxD6Joint* joint = PxD6JointCreate(*mPhysics,
        actorA, PxTransform(PxIdentity),
        actorB, PxTransform(PxIdentity));

    // Free all motion — the spring drive provides the restoring force.
    joint->setMotion(PxD6Axis::eX,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eY,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eZ,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

    // Velocity drive (accelerate flag = false) on all linear axes.
    PxD6JointDrive drive(stiffness, damping, PX_MAX_F32, false);
    joint->setDrive(PxD6Drive::eX, drive);
    joint->setDrive(PxD6Drive::eY, drive);
    joint->setDrive(PxD6Drive::eZ, drive);

    // The drive target position encodes the rest offset between the two bodies.
    joint->setDrivePosition(PxTransform(PxVec3(restLength, 0.0f, 0.0f)));

    JointComponent jc;
    jc.type       = JointComponent::Type::D6;
    jc.joint      = joint;
    jc.bodyA      = a;
    jc.bodyB      = b;
    jc.stiffness  = stiffness;
    jc.damping    = damping;
    jc.restLength = restLength;
    return jc;
}

PxD6Joint* PhysicsWorld::createDragJoint(PxRigidActor* kinematic,
                                         PxRigidActor* target,
                                         PxVec3        grabLocalPos) {
    PxD6Joint* joint = PxD6JointCreate(*mPhysics,
        kinematic, PxTransform(PxIdentity),
        target,    PxTransform(grabLocalPos));

    joint->setMotion(PxD6Axis::eX,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eY,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eZ,      PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eTWIST,  PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
    joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

    // Moderate stiffness + very high damping = snappy but jiggle-free drag.
    // Acceleration flag (true) makes the feel mass-independent.
    PxD6JointDrive drive(600.0f, 400.0f, PX_MAX_F32, true);
    joint->setDrive(PxD6Drive::eX, drive);
    joint->setDrive(PxD6Drive::eY, drive);
    joint->setDrive(PxD6Drive::eZ, drive);

    return joint;
}

void PhysicsWorld::releaseJoint(PxJoint* joint) {
    if (joint) joint->release();
}

void PhysicsWorld::clearActors(entt::registry& reg) {
    // Joints must be released before the actors they reference.
    {
        auto view = reg.view<DragStateComponent>();
        for (auto e : view) {
            auto& ds = view.get<DragStateComponent>(e);
            if (ds.dragJoint) ds.dragJoint->release();
        }
    }
    {
        auto view = reg.view<JointComponent>();
        for (auto e : view) {
            auto& jc = view.get<JointComponent>(e);
            if (jc.joint) jc.joint->release();
        }
    }
    {
        auto view = reg.view<RigidBodyComponent>();
        for (auto e : view) {
            auto& rb = view.get<RigidBodyComponent>(e);
            if (rb.actor) {
                mScene->removeActor(*rb.actor);
                rb.actor->release();
            }
        }
    }
}

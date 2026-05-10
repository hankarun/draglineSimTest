#pragma once

#include <PxPhysicsAPI.h>
#include <raylib.h>
#include <raymath.h>

inline Vector3 PxToRl(const physx::PxVec3& v) {
    return { v.x, v.y, v.z };
}

inline physx::PxVec3 RlToPx(const Vector3& v) {
    return { v.x, v.y, v.z };
}

// Build a column-major raylib Matrix from a PhysX transform (for oriented rendering).
inline Matrix PxTransformToMatrix(const physx::PxTransform& t) {
    physx::PxMat44 m(t);
    return {
        m[0][0], m[1][0], m[2][0], m[3][0],
        m[0][1], m[1][1], m[2][1], m[3][1],
        m[0][2], m[1][2], m[2][2], m[3][2],
        m[0][3], m[1][3], m[2][3], m[3][3],
    };
}

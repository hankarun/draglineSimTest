cmake_minimum_required(VERSION 3.23)
project(phyTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(unofficial-omniverse-physx-sdk CONFIG REQUIRED)
find_package(raylib CONFIG REQUIRED)
find_package(EnTT CONFIG REQUIRED)

set(SOURCES
    src/main.cpp
    src/PhysicsWorld.cpp
    src/RenderSystem.cpp
    src/InputSystem.cpp
    src/scenes/DraglineScene.cpp
    src/scenes/JointTestScene.cpp
    src/scenes/CraneScene.cpp
)

add_executable(phyTest ${SOURCES})
target_include_directories(phyTest PRIVATE src)

target_link_libraries(phyTest PRIVATE
    unofficial::omniverse-physx-sdk::sdk
    raylib
    EnTT::EnTT
)

if(MSVC)
    target_compile_definitions(phyTest PRIVATE
        NOMINMAX
        _CRT_SECURE_NO_WARNINGS
        WIN32_LEAN_AND_MEAN
    )
    target_compile_options(phyTest PRIVATE /MP)
endif()

# Copy PhysX + other runtime DLLs.
# $<TARGET_RUNTIME_DLLS:...> collects DLLs from SHARED imported targets that correctly
# set IMPORTED_LOCATION on DLL and IMPORTED_IMPLIB on .lib.
add_custom_command(TARGET phyTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:phyTest>
            $<TARGET_FILE_DIR:phyTest>
    COMMAND_EXPAND_LISTS
    COMMENT "Copying runtime DLLs (PhysX etc.)"
)

# This version of raylib's cmake config sets IMPORTED_LOCATION to the .lib, not the .dll,
# so TARGET_RUNTIME_DLLS won't catch it.  Locate and copy the DLL from vcpkg_installed.
set(VCPKG_INSTALLED_DIR "${CMAKE_BINARY_DIR}/vcpkg_installed/${VCPKG_TARGET_TRIPLET}")
add_custom_command(TARGET phyTest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${VCPKG_INSTALLED_DIR}/bin/raylib.dll"
        $<TARGET_FILE_DIR:phyTest>
    COMMENT "Copying raylib.dll"
)

# PhysX GPU acceleration DLLs (optional, only present when CUDA toolkit is installed)
if(TARGET unofficial::omniverse-physx-sdk::gpu-library)
    add_custom_command(TARGET phyTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-library>
            $<TARGET_FILE_DIR:phyTest>
        COMMENT "Copying PhysXGpu_64.dll"
    )
    add_custom_command(TARGET phyTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:unofficial::omniverse-physx-sdk::gpu-device-library>
            $<TARGET_FILE_DIR:phyTest>
        COMMENT "Copying PhysXDevice64.dll"
    )
endif()

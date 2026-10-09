// Tier: F
// Syntax-only check of the C++ side of the FFI contract (WP-4.6). No Unreal needed.
// From the repo root:
//   g++ -std=c++17 -fsyntax-only \
//     -I UE5Project/Plugins/DronePhysicsBridge/Source/DronePhysicsBridge/Public \
//     tools/cpp_ffi_check/ffi_layout_check.cpp
// All checks are static_asserts inside DroneFfiLayout.h; a mismatch is a compile error.
// Lives outside the plugin Source/ tree so UnrealBuildTool never compiles it.

#include "DroneFfiLayout.h"

static_assert(kDroneFfiExpectedInterfaceVersion > 0, "interface version must be positive");

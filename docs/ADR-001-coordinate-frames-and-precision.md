# ADR-001: Coordinate frames, quaternions and numeric precision

Status: Proposed, UNREVIEWED by a human (the Completion Plan text says "Accepted"; change this only after human review, exit gate 15)
Date: 2026-10-08    File created: 2026-10-09    Owners: Lead Architect

Source: Phase 2 Completion Plan, Part 4.1. Edit only through a reviewed pull request.

## Context

The physics core (Rust) and the viewer (Unreal Engine 5) use different
coordinate systems. Mixing them silently causes mirrored motion and lost precision.

## Decisions

1. Physics frame is local North-East-Down (NED), right handed, metres.
   Origin is a fixed WGS84 geodetic point set once per scenario.
2. Position is f64. Integration, ISA and time step are f64 (WP-1).
   Velocity, orientation and angular velocity may cross the FFI as f32.
3. Quaternions are stored (x, y, z, w), unit length, body to NED.
4. Body frame: X forward, Y right, Z down.
   Positive roll = right wing down. Positive pitch = nose up. Positive yaw = nose right.
5. Conversion to Unreal happens in ONE place, the bridge class:
   position (N, E, D) metres -> (N*100, E*100, -D*100) cm;
   rotation: q_ue = (-x, -y, z, w) (verified by engine tests in WP-5).
6. Fixed physics step 0.002 s. Unreal frame time is never passed to the core.
7. Any change to a repr(C) struct bumps ffi_get_interface_version.

## Consequences

Unreal world coordinates must never enter the Rust core.
Tests must cover each conversion with a known pose.

## Open items

Behaviour of flight-candidate runs when a surrogate is outside its envelope.

## Repo conformance at creation (informative, not part of the decision)

Checked 2026-10-09 by reading the source. Nothing below was run in the engine.
`D<n>` refers to Decision `<n>`. Item numbers refer to "Remaining work (A)" in `dev-docs/PHASE2-REALITY-CHECK.md`.

- D1: the WGS84 origin appears only in doc comments (`physics_core/src/types.rs:10,16`). No code sets or uses it.
- D2: PARTLY FIXED by item (A)8 (2026-10-09): net force, net torque and airspeed now reach the integrator in
  f64 (`integrator::step_rk4` takes `[f64; 3]`; `lib.rs` `airspeed_mps`). Still f32: the thrust input and
  output of `aero::get_thrust` (`lib.rs` thrust loop, item (A)10), `aero::get_drag` output and the
  `load_proxy` (surrogate API, items (A)10 and (A)19).
- D3: FIXED by item (A)15 (2026-10-09): the stale root `physics_core/drone_ffi.h` was deleted; CI diff-checks the generated header (step not yet run on real CI).
- D4: FIXED by item (A)7 (2026-10-09): the mixer yaw column now follows the shipped `motor_spin [1, -1, -1, 1]`
  (FR CCW, FL CW, RR CW, RL CCW seen from above; spin +1 = reaction torque about body +Z). A positive yaw command
  gives a positive yaw torque (hand-calculated tests). The mapping to the physical motors is TODO(HUMAN-VERIFY: matches real airframe), B-3.
- D5: `DronePhysicsBridge.cpp:66` gives (N*100, E*100, -D*100) and `:71` gives (-x, -y, z, w), as decided.
  Neither is tested in the engine. `UE5Project/Source/UAV_Simulator/DroneTelemetryComponent.cpp:96-98`
  converts force arrows ad hoc, outside the one bridge function. Item (A)14.
- D7: the interface version is 5 (`physics_core/src/lib.rs:22`); the rule has been followed so far.

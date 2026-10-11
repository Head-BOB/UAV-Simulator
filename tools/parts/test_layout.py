# Tier: D
"""Hand-calculated tests for layout.py (positions, frame split, slots)."""
import math
import unittest

import layout as L
import partsdb as P

RECS, _ = P.load_records(P.DEFAULT_DB)
FRAME = RECS["ref_frame_quad_400"]


class MotorTests(unittest.TestCase):
    def test_motor_positions_and_order(self):
        # FR, FL, RR, RL with x forward, y right, s = 0.4 -> (+-0.2, +-0.2): same order as the shipped vehicle_config.json
        self.assertEqual(L.motor_xy(0.4), [(0.2, 0.2), (0.2, -0.2), (-0.2, 0.2), (-0.2, -0.2)])

    def test_spin_pattern_is_the_shipped_one(self):
        self.assertEqual(L.MOTOR_SPIN, [1.0, -1.0, -1.0, 1.0])
        # diagonal motors spin the same way, adjacent motors opposite (yaw balance at equal thrust)
        self.assertEqual(sum(L.MOTOR_SPIN), 0.0)
        self.assertEqual(L.MOTOR_SPIN[0], L.MOTOR_SPIN[3])
        self.assertEqual(L.MOTOR_SPIN[1], L.MOTOR_SPIN[2])


class ArmTests(unittest.TestCase):
    def test_tube_section_of_the_reference_frame(self):
        s = L.arm_section(FRAME)
        self.assertEqual(s["kind"], "tube")
        self.assertAlmostEqual(s["half_thickness_m"], 0.008, places=12)
        self.assertAlmostEqual(s["area_m2"], math.pi / 4 * 8.7e-5, places=15)

    def test_plate_section_of_the_fpv_frame(self):
        s = L.arm_section(RECS["holybro_kopis2_220_frame"])
        self.assertEqual(s["kind"], "plate")
        self.assertAlmostEqual(s["half_thickness_m"], 0.0025, places=12)
        self.assertAlmostEqual(s["area_m2"], 0.025 * 0.005, places=15)


class FrameSplitTests(unittest.TestCase):
    def test_reference_frame_mass_split(self):
        # A = pi/4 * 8.7e-5 = 6.832964e-5 m^2; arm = 1600 * A * 0.282843 = 0.0309225 kg; four arms 0.12369; body 0.35 - 0.12369 = 0.22631
        bodies, info = L.frame_bodies(FRAME, L.arm_section(FRAME))
        arm = 1600.0 * (math.pi / 4 * 8.7e-5) * 0.282843
        self.assertAlmostEqual(info["arm_mass_each_kg"], arm, places=9)
        self.assertAlmostEqual(info["body_mass_kg"], 0.35 - 4 * arm, places=9)
        self.assertAlmostEqual(sum(b.mass_kg for b in bodies), 0.35, places=12)  # the split never changes the frame mass
        self.assertEqual(len(bodies), 5)

    def test_arm_rods_point_at_the_motors(self):
        bodies, _ = L.frame_bodies(FRAME, L.arm_section(FRAME))
        # first arm toward (+,+): centre at half the arm length along (1,1)/sqrt2 = 0.141421 * (0.7071, 0.7071) = (0.1, 0.1)
        self.assertAlmostEqual(bodies[1].position_m[0], 0.1, places=5)
        self.assertAlmostEqual(bodies[1].position_m[1], 0.1, places=5)
        self.assertEqual(bodies[1].position_m[2], 0.0)

    def test_inconsistent_arm_section_refused(self):
        heavy = dict(FRAME, mass_kg={"value": 0.1, "provenance": FRAME["mass_kg"]["provenance"]})
        with self.assertRaises(L.LayoutError):
            L.frame_bodies(heavy, L.arm_section(heavy))  # 0.1237 kg of arms >= 90 % of 0.1 kg

    def test_missing_central_box_refused(self):
        no_box = {k: v for k, v in FRAME.items() if k != "central_body_box_m"}
        with self.assertRaises(L.LayoutError):
            L.frame_bodies(no_box, L.arm_section(no_box))


class CornerTests(unittest.TestCase):
    def test_heights_above_the_arm(self):
        motor, prop, esc = RECS["tmotor_mn3508_kv700"], RECS["apc_12x6e"], RECS["tmotor_air_40a_2_6s"]
        bodies, z_prop, _ = L.corner_bodies(motor, esc, prop, 0.4, 0.008, True)
        # motor centre: -(0.008 + 0.0265/2) = -0.02125; prop plane: -(0.008 + 0.0265 + 0.015/2) = -0.042
        self.assertAlmostEqual(z_prop, -0.042, places=12)
        by = {b.name: b for b in bodies}
        self.assertAlmostEqual(by["motor_0"].position_m[2], -0.02125, places=12)
        self.assertAlmostEqual(by["esc_0"].position_m[2], -0.02125, places=12)
        self.assertEqual(len(bodies), 12)  # 4 motors + 4 props + 4 ESCs

    def test_four_in_one_has_no_corner_escs(self):
        motor, prop, esc = RECS["tmotor_mn3508_kv700"], RECS["apc_12x6e"], RECS["holybro_tekko32_f4_4in1_50a"]
        bodies, _, _ = L.corner_bodies(motor, esc, prop, 0.4, 0.008, False)
        self.assertEqual(len(bodies), 8)


class SlotTests(unittest.TestCase):
    CTX = {"body_h_m": 0.1, "body_x_m": 0.2, "battery_h_m": 0.034}

    def item(self, rid, role, slot, bbox):
        return {"id": rid, "role": role, "slot": slot, "bbox": bbox}

    def test_two_parts_side_by_side_centred(self):
        # widths 0.02 and 0.04 -> total 0.06, start -0.03: centres at -0.03 + 0.01 = -0.02 and -0.03 + 0.02 + 0.02 = 0.01
        out = L.place_slotted([self.item("b", "receiver", "frame_plate", (0.04, 0.01, 0.01)),
                               self.item("a", "flight_controller", "frame_plate", (0.02, 0.01, 0.01))], self.CTX)
        self.assertAlmostEqual(out["a"][0], -0.02, places=12)   # flight_controller sorts before receiver
        self.assertAlmostEqual(out["b"][0], 0.01, places=12)
        # top plate sits above the body: z = -0.1/2 - 0.01/2 = -0.055 (Z down, so negative is up)
        self.assertAlmostEqual(out["a"][2], -0.055, places=12)

    def test_mast_front_belly_power(self):
        c = self.CTX
        out = L.place_slotted([self.item("g", "gps", "mast", (0.05, 0.05, 0.0144)), self.item("c", "camera", "front_mount", (0.019, 0.019, 0.027)),
                               self.item("s", "gimbal", "belly_mount", (0.055, 0.055, 0.07)), self.item("p", "pdb", "power_input", (0.068, 0.05, 0.01))], c)
        self.assertAlmostEqual(out["g"][2], -0.05 - 0.05 - 0.0144 / 2, places=12)         # -h/2 - mast - c/2
        self.assertAlmostEqual(out["c"][0], 0.1 + 0.019 / 2, places=12)                    # box_x/2 + a/2
        self.assertAlmostEqual(out["s"][2], 0.05 + 0.034 + 0.035, places=12)               # h/2 + battery_h + c/2 (below the battery)
        self.assertAlmostEqual(out["p"][2], 0.05 - 0.005, places=12)                       # inside the bottom of the body

    def test_front_mount_parts_stack_along_y_not_x(self):
        # two cameras 0.02 wide in y: total 0.04, centres at y = -0.01 and +0.01; both at the same x = box_x/2 + a/2 = 0.1 + 0.0095
        out = L.place_slotted([self.item("a", "camera", "front_mount", (0.019, 0.02, 0.02)),
                               self.item("b", "gimbal", "front_mount", (0.019, 0.02, 0.02))], self.CTX)
        self.assertAlmostEqual(out["a"][1], -0.01, places=12)
        self.assertAlmostEqual(out["b"][1], 0.01, places=12)
        self.assertAlmostEqual(out["a"][0], 0.1095, places=12)
        self.assertAlmostEqual(out["b"][0], 0.1095, places=12)

    def test_unknown_slot_refused(self):
        with self.assertRaises(L.LayoutError):
            L.place_slotted([self.item("x", "gps", "roof_rack", (0.01, 0.01, 0.01))], self.CTX)

    def test_every_seed_slot_is_known(self):
        for rid, rec in RECS.items():
            if "mount_slot" in rec:
                self.assertIn(rec["mount_slot"], L.SLOTS, rid)


if __name__ == "__main__":
    unittest.main()

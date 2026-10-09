# Tier: V
"""Prepare the Sketchfab 'Animated Drone' (hartwelkisaka, CC-BY-4.0) for Unreal.

Run headless:
    blender -b --python prepare_drone.py -- "C:\\DroneAssets\\scene.gltf" "C:\\DroneAssets\\Drone.fbx"

What it does (no manual renaming):
  1. Imports the glTF (needs scene.bin and textures/ next to the .gltf).
  2. Strips the '.NN_NN' suffix Sketchfab adds to bone names
     (prop_1_jnt.34_34 -> prop_1_jnt).
  3. Checks every bone the Unreal code needs exists, and prints a report.
  4. Removes the three baked animations (hover, exploded_view, step_by_step).
  5. Exports FBX: armature + meshes, Z up, forward -Y, embedded textures.
TODO(HUMAN-VERIFY): this script has not been run by its author. Read the
printed report; if a REQUIRED bone is MISSING, stop and send the report.
"""
import re
import sys

import bpy

REQUIRED_BONES = (
    [f"prop_{i}_jnt" for i in range(1, 5)]
    + [f"prop_bolt_nut_{i}_jnt" for i in range(1, 5)]
    + [f"prop_bolt_cap_{i}_jnt" for i in range(1, 5)]
    + [f"prop_motor_bolt_{i}_jnt" for i in range(1, 5)]
    + [f"motor_{i}_jnt" for i in range(1, 5)]
    + ["CameraHandle_jnt", "camera_jnt"]
)
SUFFIX = re.compile(r"\.\d+_\d+$")


def main() -> int:
    argv = sys.argv
    args = argv[argv.index("--") + 1:] if "--" in argv else []
    if len(args) != 2:
        print("usage: blender -b --python prepare_drone.py -- in.gltf out.fbx")
        return 2
    src, dst = args

    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.gltf(filepath=src)

    armatures = [o for o in bpy.data.objects if o.type == "ARMATURE"]
    meshes = [o for o in bpy.data.objects if o.type == "MESH"]
    print(f"REPORT: {len(armatures)} armature(s), {len(meshes)} mesh object(s)")
    if len(armatures) != 1:
        print("ERROR: expected exactly one armature")
        return 1

    arm = armatures[0]
    renamed = 0
    for bone in arm.data.bones:
        clean = SUFFIX.sub("", bone.name)
        if clean != bone.name:
            bone.name = clean
            renamed += 1
    print(f"REPORT: renamed {renamed} bones")

    names = {b.name for b in arm.data.bones}
    missing = [b for b in REQUIRED_BONES if b not in names]
    for b in REQUIRED_BONES:
        print(f"  {'OK     ' if b in names else 'MISSING'} {b}")
    if missing:
        print(f"ERROR: {len(missing)} required bone(s) missing: {missing}")
        return 1

    # Report prop hub positions (Blender: X right, Y forward, Z up, metres).
    for i in range(1, 5):
        pb = arm.data.bones[f"prop_{i}_jnt"]
        head = arm.matrix_world @ pb.head_local
        print(f"REPORT: prop_{i}_jnt hub at x={head.x:.4f} y={head.y:.4f} z={head.z:.4f}")

    for action in list(bpy.data.actions):
        bpy.data.actions.remove(action)
    if arm.animation_data:
        arm.animation_data_clear()

    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.export_scene.fbx(
        filepath=dst,
        use_selection=True,
        object_types={"ARMATURE", "MESH"},
        add_leaf_bones=False,
        bake_anim=False,
        axis_forward="-Y",
        axis_up="Z",
        apply_scale_options="FBX_SCALE_UNITS",
        path_mode="COPY",
        embed_textures=True,
        mesh_smooth_type="FACE",
    )
    print(f"REPORT: wrote {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

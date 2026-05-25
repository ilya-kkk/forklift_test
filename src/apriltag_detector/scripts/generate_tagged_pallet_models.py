#!/usr/bin/env python3
from pathlib import Path


PALLET_TAGS = [
    ("pallet_b_south_00", 0),
    ("pallet_b_south_01", 1),
    ("pallet_b_south_02", 2),
    ("pallet_b_south_03", 3),
    ("pallet_b_south_04", 4),
    ("pallet_b_south_05", 5),
    ("pallet_b_south_06", 6),
    ("pallet_b_south_07", 7),
    ("pallet_b_south_08", 8),
    ("pallet_b_south_09", 9),
    ("pallet_b_south_10", 10),
    ("pallet_b_south_11", 11),
    ("pallet_b_north_00", 12),
    ("pallet_b_north_01", 13),
    ("pallet_b_north_02", 14),
    ("pallet_b_north_03", 15),
    ("pallet_b_north_04", 16),
    ("pallet_b_north_05", 17),
    ("pallet_b_north_06", 18),
    ("pallet_b_north_07", 19),
    ("pallet_b_north_09", 20),
    ("pallet_b_north_10", 21),
    ("pallet_b_north_11", 22),
]

TAG36H11_CODES = [
    0x0000000D7E00984B,
    0x0000000DDA664CA7,
    0x0000000DC4A1C821,
    0x0000000E17B470E9,
    0x0000000EF91D01B1,
    0x0000000F429CDD73,
    0x000000005DA29225,
    0x00000001106CBA43,
    0x0000000223BED79D,
    0x000000021F51213C,
    0x000000033EB19CA6,
    0x00000003F76EB0F8,
    0x0000000469A97414,
    0x000000045DCFE0B0,
    0x00000004A6465F72,
    0x000000051801DB96,
    0x00000005EB946B4E,
    0x000000068A7CC2EC,
    0x00000006F0BA2652,
    0x000000078765559D,
    0x000000087B83D129,
    0x000000086CC4A5C5,
    0x00000008B64DF90F,
]

TAG36H11_BIT_COORDS = [
    (1, 1),
    (2, 1),
    (3, 1),
    (4, 1),
    (5, 1),
    (2, 2),
    (3, 2),
    (4, 2),
    (3, 3),
    (6, 1),
    (6, 2),
    (6, 3),
    (6, 4),
    (6, 5),
    (5, 2),
    (5, 3),
    (5, 4),
    (4, 3),
    (6, 6),
    (5, 6),
    (4, 6),
    (3, 6),
    (2, 6),
    (5, 5),
    (4, 5),
    (3, 5),
    (4, 4),
    (1, 6),
    (1, 5),
    (1, 4),
    (1, 3),
    (1, 2),
    (2, 5),
    (2, 4),
    (2, 3),
    (3, 4),
]

TOTAL_WIDTH_MODULES = 10
WIDTH_AT_BORDER_MODULES = 8
TOTAL_TAG_SIZE_M = 0.30
MODULE_SIZE_M = TOTAL_TAG_SIZE_M / TOTAL_WIDTH_MODULES
DETECTOR_TAG_SIZE_M = MODULE_SIZE_M * WIDTH_AT_BORDER_MODULES
FACE_X_M = 0.605
PLATE_X_M = 0.602
TAG_CENTER_Z_M = 0.50


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def white_cells_for_code(code: int) -> set[tuple[int, int]]:
    cells = set()
    for offset in range(TOTAL_WIDTH_MODULES - 1):
        cells.add((offset, 0))
        cells.add((TOTAL_WIDTH_MODULES - 1, offset))
        cells.add((offset + 1, TOTAL_WIDTH_MODULES - 1))
        cells.add((0, offset + 1))

    for bit_index, (bit_x, bit_y) in enumerate(TAG36H11_BIT_COORDS):
        bit = (code >> (35 - bit_index)) & 1
        if bit == 1:
            cells.add((bit_x + 1, bit_y + 1))

    return cells


def contiguous_runs(row_cells: list[int]) -> list[tuple[int, int]]:
    if not row_cells:
        return []

    runs = []
    start = row_cells[0]
    previous = row_cells[0]
    for cell in row_cells[1:]:
        if cell == previous + 1:
            previous = cell
            continue
        runs.append((start, previous))
        start = cell
        previous = cell
    runs.append((start, previous))
    return runs


def make_plate_visual(name: str, face_sign: int) -> str:
    x = PLATE_X_M * face_sign
    return f"""      <visual name="{name}_black_plate">
        <pose>{x:.3f} 0 {TAG_CENTER_Z_M:.3f} 0 0 0</pose>
        <geometry>
          <box>
            <size>0.002 {TOTAL_TAG_SIZE_M:.3f} {TOTAL_TAG_SIZE_M:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>"""


def make_white_visuals(prefix: str, face_sign: int, cells: set[tuple[int, int]]) -> str:
    visuals = []
    x = FACE_X_M * face_sign
    for grid_y in range(TOTAL_WIDTH_MODULES):
        row = sorted(grid_x for grid_x, y in cells if y == grid_y)
        for run_start, run_end in contiguous_runs(row):
            run_center = (run_start + run_end) / 2.0
            run_len = run_end - run_start + 1
            y = face_sign * (run_center - 4.5) * MODULE_SIZE_M
            z = TAG_CENTER_Z_M + (4.5 - grid_y) * MODULE_SIZE_M
            visuals.append(
                f"""      <visual name="{prefix}_white_r{grid_y:02d}_{run_start:02d}_{run_end:02d}">
        <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>
        <geometry>
          <box>
            <size>0.003 {run_len * MODULE_SIZE_M:.3f} {MODULE_SIZE_M:.3f}</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>"""
            )
    return "\n".join(visuals)


def make_tag_visuals(tag_id: int) -> str:
    cells = white_cells_for_code(TAG36H11_CODES[tag_id])
    parts = [
        make_plate_visual("apriltag_front", 1),
        make_white_visuals("apriltag_front", 1, cells),
        make_plate_visual("apriltag_back", -1),
        make_white_visuals("apriltag_back", -1, cells),
    ]
    return "\n".join(parts)


def make_model_config(model_name: str, pallet_name: str, tag_id: int) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Codex</name>
    <email>codex@example.com</email>
  </author>
  <description>Euro pallet with tag36h11 ID {tag_id} for {pallet_name}.</description>
</model>
"""


def main() -> None:
    root = repo_root()
    base_model = root / "src" / "forklift_demo" / "models" / "euro_pallet"
    base_sdf = (base_model / "model.sdf").read_text()
    models_root = root / "src" / "forklift_demo" / "models"

    for pallet_name, tag_id in PALLET_TAGS:
        model_name = f"euro_pallet_tag36h11_{tag_id:02d}"
        model_dir = models_root / model_name
        model_dir.mkdir(parents=True, exist_ok=True)

        model_sdf = base_sdf.replace(
            '<model name="euro_pallet">', f'<model name="{model_name}">', 1
        )
        tag_visuals = make_tag_visuals(tag_id)
        model_sdf = model_sdf.replace(
            "      <visual name=\"marker_box\">",
            f"{tag_visuals}\n      <visual name=\"marker_box\">",
            1,
        )
        (model_dir / "model.sdf").write_text(model_sdf)
        (model_dir / "model.config").write_text(
            make_model_config(model_name, pallet_name, tag_id)
        )

    print(
        f"Generated {len(PALLET_TAGS)} tagged pallet models "
        f"with detector tag size {DETECTOR_TAG_SIZE_M:.3f} m"
    )


if __name__ == "__main__":
    main()

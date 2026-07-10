#!/usr/bin/env python3
import argparse
import math
import random
from pathlib import Path


START_MARKER = "    <!-- BEGIN GENERATED OBSTACLES -->"
END_MARKER = "    <!-- END GENERATED OBSTACLES -->"


def fmt(value):
    return f"{value:.3f}".rstrip("0").rstrip(".")


def random_position(rng, x_range, y_range, keepout_radius, max_tries=1000):
    for _ in range(max_tries):
        x = rng.uniform(*x_range)
        y = rng.uniform(*y_range)
        if math.hypot(x, y) >= keepout_radius:
            return x, y

    raise RuntimeError(
        "Could not place an obstacle outside the keepout radius. "
        "Increase the area or reduce --keepout-radius."
    )


def material_xml(color):
    rgba = " ".join(fmt(value) for value in color)
    return f"""          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>"""


def box_xml(index, x, y, yaw, width, depth, height, color):
    z = height / 2.0
    return f"""    <model name="random_box_{index}">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{fmt(width)} {fmt(depth)} {fmt(height)}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{fmt(width)} {fmt(depth)} {fmt(height)}</size>
            </box>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""


def cylinder_xml(index, x, y, yaw, radius, height, color):
    z = height / 2.0
    return f"""    <model name="random_cylinder_{index}">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{fmt(radius)}</radius>
              <length>{fmt(height)}</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{fmt(radius)}</radius>
              <length>{fmt(height)}</length>
            </cylinder>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""


def generate_obstacles(args):
    rng = random.Random(args.seed)
    x_range = (args.x_min, args.x_max)
    y_range = (args.y_min, args.y_max)
    obstacles = []

    for index in range(1, args.count + 1):
        shape = rng.choice(("box", "cylinder"))
        x, y = random_position(rng, x_range, y_range, args.keepout_radius)
        yaw = rng.uniform(-math.pi, math.pi)
        height = rng.uniform(args.height_min, args.height_max)
        color = (rng.uniform(0.55, 1.0), rng.uniform(0.05, 0.35), rng.uniform(0.05, 0.25), 1.0)

        if shape == "box":
            width = rng.uniform(args.box_min, args.box_max)
            depth = rng.uniform(args.box_min, args.box_max)
            obstacles.append(box_xml(index, x, y, yaw, width, depth, height, color))
        else:
            radius = rng.uniform(args.radius_min, args.radius_max)
            obstacles.append(cylinder_xml(index, x, y, yaw, radius, height, color))

    return "\n\n".join(obstacles)


def replace_generated_block(world_path, generated_xml):
    text = world_path.read_text()
    start = text.index(START_MARKER)
    end = text.index(END_MARKER, start)
    replacement = f"{START_MARKER}\n{generated_xml}\n{END_MARKER}"
    return text[:start] + replacement + text[end + len(END_MARKER):]


def parse_args():
    parser = argparse.ArgumentParser(
        description="Regenerate the obstacle block in my_world.sdf."
    )
    parser.add_argument(
        "--world",
        default=Path(__file__).with_name("my_world.sdf"),
        type=Path,
        help="SDF file containing the generated obstacle markers.",
    )
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--x-min", type=float, default=5.0)
    parser.add_argument("--x-max", type=float, default=60.0)
    parser.add_argument("--y-min", type=float, default=-20.0)
    parser.add_argument("--y-max", type=float, default=20.0)
    parser.add_argument("--height-min", type=float, default=2.0)
    parser.add_argument("--height-max", type=float, default=12.0)
    parser.add_argument("--box-min", type=float, default=1.0)
    parser.add_argument("--box-max", type=float, default=6.0)
    parser.add_argument("--radius-min", type=float, default=0.6)
    parser.add_argument("--radius-max", type=float, default=3.0)
    parser.add_argument("--keepout-radius", type=float, default=6.0)
    return parser.parse_args()


def main():
    args = parse_args()
    generated_xml = generate_obstacles(args)
    updated_world = replace_generated_block(args.world, generated_xml)
    args.world.write_text(updated_world)
    print(f"Wrote {args.count} random obstacles to {args.world}")


if __name__ == "__main__":
    main()

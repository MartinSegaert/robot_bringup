#!/usr/bin/env python3
import argparse
import math
import random
from pathlib import Path


START_MARKER = "    <!-- BEGIN GENERATED OBSTACLES -->"
END_MARKER = "    <!-- END GENERATED OBSTACLES -->"


def fmt(value):
    return f"{value:.3f}".rstrip("0").rstrip(".")


def random_position(x_range: list[float], y_range: list[float], z_range: list[float] = [0.0, 0.0], rng: random.Random|None = None):
    if rng is None:
      x = random.uniform(*x_range)
      y = random.uniform(*y_range)
      z = random.uniform(*z_range)
    else:
      x = rng.uniform(*x_range)
      y = rng.uniform(*y_range)
      z = rng.uniform(*z_range)
    return x, y, z


def material_xml(color):
    rgba = " ".join(fmt(value) for value in color)
    return f"""          <material>
            <ambient>{rgba}</ambient>
            <diffuse>{rgba}</diffuse>
          </material>"""


def box_xml(name: str, x, y, yaw, length, width, height, color):
    z = height / 2.0
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{fmt(x)} {fmt(y)} {fmt(z)} 0 0 {fmt(yaw)}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{fmt(length)} {fmt(width)} {fmt(height)}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{fmt(length)} {fmt(width)} {fmt(height)}</size>
            </box>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""


def cylinder_xml(name, x, y, yaw, radius, height, color):
    z = height / 2.0
    return f"""    <model name="{name}">
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


def plane_xml(name: str, position: tuple, length, width, normal: tuple, color):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{fmt(position[0])} {fmt(position[1])} {fmt(position[2])} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>{fmt(normal[0])} {fmt(normal[1])} {fmt(normal[2])}</normal>
              <size>{fmt(length)} {fmt(width)}</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>{fmt(normal[0])} {fmt(normal[1])} {fmt(normal[2])}</normal>
              <size>{fmt(length)} {fmt(width)}</size>
            </plane>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""

def triangle_xml(name: str, position: tuple, orientation: tuple, height, p_1: tuple, p_2: tuple, p_3: tuple, color):
    orientation = [x * math.pi / 180 for x in orientation]  # convert degrees to * math.pi / 180 # type: ignore
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{fmt(position[0])} {fmt(position[1])} {fmt(position[2])} {fmt(orientation[0])} {fmt(orientation[1])} {fmt(orientation[2])}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <polyline>
              <height>{fmt(height)}</height>
              <point>{fmt(p_1[0])} {fmt(p_1[1])}</point>
              <point>{fmt(p_2[0])} {fmt(p_2[1])}</point>
              <point>{fmt(p_3[0])} {fmt(p_3[1])}</point>
            </polyline>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <polyline>
              <height>{fmt(height)}</height>
              <point>{fmt(p_1[0])} {fmt(p_1[1])}</point>
              <point>{fmt(p_2[0])} {fmt(p_2[1])}</point>
              <point>{fmt(p_3[0])} {fmt(p_3[1])}</point>
            </polyline>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""

def trapezoid_xml(name: str, p_1: tuple, p_2: tuple, p_3: tuple, p_4: tuple, height, pitch, color):
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>0 0 0 0 {fmt(pitch)} 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <polyline>
              <height>{fmt(height)}</height>
              <point>{fmt(p_1[0])} {fmt(p_1[1])}</point>
              <point>{fmt(p_2[0])} {fmt(p_2[1])}</point>
              <point>{fmt(p_3[0])} {fmt(p_3[1])}</point>
              <point>{fmt(p_4[0])} {fmt(p_4[1])}</point>
            </polyline>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <polyline>
              <height>{fmt(height)}</height>
              <point>{fmt(p_1[0])} {fmt(p_1[1])}</point>
              <point>{fmt(p_2[0])} {fmt(p_2[1])}</point>
              <point>{fmt(p_3[0])} {fmt(p_3[1])}</point>
              <point>{fmt(p_4[0])} {fmt(p_4[1])}</point>
            </polyline>
          </geometry>
{material_xml(color)}
        </visual>
      </link>
    </model>"""

def get_random_color():
    return (random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), 1.0)

def generate_hill(
    hill_id: int,
    offset: tuple = (0, 0),
    height_hill: float|None = None,
    width_hill: float|None = None,
    length_ramp_up: float|None = None,
    length_plane: float|None = None,
    length_ramp_down: float|None = None,
    n_obstacles: int|None = None
  ) -> list:
    
  obstacles = []

  if height_hill is None:
      height_hill = random.uniform(5, 20)
  if width_hill is None:
      width_hill = random.uniform(50, 250)
  if length_ramp_up is None:
      length_ramp_up = random.uniform(10, 40)
  if length_plane is None:
      length_plane = random.uniform(30, 150)
  if length_ramp_down is None:
      length_ramp_down = random.uniform(10, 40)
  if n_obstacles is None:
      n_obstacles = random.randint(0, 10)

  ## Hill
  # Hill up
  color = get_random_color()
  obstacles.append(triangle_xml(
      name=f'hill_up_{hill_id}',
      position=(offset[0], offset[1] + width_hill/2, 0),
      orientation=(90, 0, 0),
      height=width_hill,
      p_1=(0, 0), p_2=(length_ramp_up, 0), p_3=(length_ramp_up, height_hill),
      color=color)
  )

  # plane
  color = get_random_color()
  obstacles.append(box_xml(
      name=f"plane_{hill_id}",
      x=offset[0]+length_ramp_up+length_plane/2, y=offset[1],
      yaw=0,
      length=length_plane, width=width_hill,
      height=height_hill,
      color=color
    )
  )

  # Hill down
  color = get_random_color()
  obstacles.append(triangle_xml(
      name=f'hill_down_{hill_id}',
      position=(offset[0] + length_ramp_up + length_plane + length_ramp_down, offset[1] - width_hill/2, 0),
      orientation=(90, 0, 180),
      height=width_hill,
      p_1=(0, 0), p_2=(length_ramp_down, 0), p_3=(length_ramp_down, height_hill),
      color=color)
  )

  # Obstacles
  color = get_random_color()
  for i in range(n_obstacles):
      position = random_position(
          x_range=[offset[0] + length_ramp_up + 1/6 * length_plane, offset[0] + length_ramp_up + 5/6*length_plane],
          y_range=[offset[1] - 3/4*width_hill/2, offset[1] + 3/4*width_hill/2]
      )
      obstacles.append(
          box_xml(
              name=f"obstacle_{i}_{hill_id}",
              x=position[0], y=position[1],
              yaw=random.uniform(0, 360),
              length=random.uniform(1, 7), width=random.uniform(1, 7),
              height=height_hill+random.uniform(1, 7),
              color=color
          )
      )

  return obstacles


def generate_obstacles():
    obstacles = []
    # hill 1
    height_hill = 10
    width_hill = 250
    length_ramp_up = 10
    length_plane = 30
    length_ramp_down = 15
    obstacles.extend(
        generate_hill(
          hill_id=1, offset=(5, 0),
          height_hill=height_hill, width_hill=width_hill, length_ramp_up=length_ramp_up, length_plane=length_plane, length_ramp_down=length_ramp_down,
          n_obstacles=10
      )
    )

    length_hill_1 = length_ramp_up + length_plane + length_ramp_down
    gap_hills = 50

    # hill 2
    height_hill = 20
    length_ramp_up = 20
    length_plane = 40
    length_ramp_down = 5
    obstacles.extend(
        generate_hill(
            hill_id=2, offset=(length_hill_1 + gap_hills, 0),
            height_hill=height_hill, width_hill=width_hill, length_ramp_up=length_ramp_up, length_plane=length_plane, length_ramp_down=length_ramp_down,
            n_obstacles=10
      )
    )
    
    # Building (between hills)
    obstacles.append(
        box_xml(
            name="building",
            x=length_hill_1 + gap_hills/2, y= -3/4*width_hill/2,
            yaw=random.uniform(0, 90),
            length=gap_hills/2, width=gap_hills/2,
            height=random.uniform(30, 50),
            color=get_random_color()
        )
    )

    return "\n\n".join(obstacles)


def replace_generated_block(world_path, generated_xml):
    text = world_path.read_text()
    start = text.index(START_MARKER)
    end = text.index(END_MARKER, start)
    replacement = f"{START_MARKER}\n{generated_xml}\n{END_MARKER}"
    return text[:start] + replacement + text[end + len(END_MARKER):]


def main():
    world_path = Path(__file__).with_name("crete.sdf")
    generated_xml = generate_obstacles()
    updated_world = replace_generated_block(world_path, generated_xml)
    world_path.write_text(updated_world) # type: ignore
    print(f"Wrote obstacles to {world_path}")


if __name__ == "__main__":
    main()

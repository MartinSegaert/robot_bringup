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

def generate_obstacles_hill_1(offset: tuple = (0, 0)):
    obstacles = []

    height_hill = 10
    width_hill = 100
    length_hill_up = 10
    length_plane = 30
    length_hill_down = 15

    hill_id = 1

    # Hill up
    color = get_random_color()
    obstacles.append(triangle_xml(
        name=f'hill_up_{hill_id}',
        position=(offset[0], offset[1] + width_hill/2, 0),
        orientation=(90, 0, 0),
        height=width_hill,
        p_1=(0, 0), p_2=(length_hill_up, 0), p_3=(length_hill_up, height_hill),
        color=color)
    )

    # plane
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"plane_{hill_id}",
        x=offset[0]+length_hill_up+length_plane/2, y=offset[1],
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
        position=(offset[0] + length_hill_up + length_plane + length_hill_down, offset[1] - width_hill/2, 0),
        orientation=(90, 0, 180),
        height=width_hill,
        p_1=(0, 0), p_2=(length_hill_down, 0), p_3=(length_hill_down, height_hill),
        color=color)
    )

    # obstacle 1
    x_ratio_plane = 1/3
    y_pos = 0
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_1_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=3, width=5,
        height=height_hill+5,
        color=color
      )
    )

    # obstacle 2
    x_ratio_plane = 2/3
    y_pos = 4
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_2_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=5, width=3,
        height=height_hill+4,
        color=color
      )
    )

    # obstacle 3
    x_ratio_plane = 2/3
    y_pos = -6
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_3_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=3, width=7,
        height=height_hill+3,
        color=color
      )
    )

    # obstacle 4
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_4_{hill_id}",
        x=offset[0]+length_hill_up+length_plane + length_hill_down/2,
        y=offset[1],
        yaw=0,
        length=3, width=10,
        height=height_hill,
        color=color
      )
    )

    return obstacles
    
def generate_obstacles_hill_2(offset: tuple = (0, 0)):
    obstacles = []

    height_hill = 20
    width_hill = 100
    length_hill_up = 20
    length_plane = 30
    length_hill_down = 5

    hill_id = 2

    # Hill up
    color = get_random_color()
    obstacles.append(triangle_xml(
        name=f'hill_up_{hill_id}',
        position=(offset[0], offset[1] + width_hill/2, 0),
        orientation=(90, 0, 0),
        height=width_hill,
        p_1=(0, 0), p_2=(length_hill_up, 0), p_3=(length_hill_up, height_hill),
        color=color)
    )

    # plane
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"plane_{hill_id}",
        x=offset[0]+length_hill_up+length_plane/2, y=offset[1],
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
        position=(offset[0] + length_hill_up + length_plane + length_hill_down, offset[1] - width_hill/2, 0),
        orientation=(90, 0, 180),
        height=width_hill,
        p_1=(0, 0), p_2=(length_hill_down, 0), p_3=(length_hill_down, height_hill),
        color=color)
    )

    # obstacle 1
    x_ratio_plane = 1/3
    y_pos = 0
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_1_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=3, width=5,
        height=height_hill+5,
        color=color
      )
    )

    # obstacle 2
    x_ratio_plane = 1/3
    y_pos = 15
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_2_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=5, width=3,
        height=height_hill+4,
        color=color
      )
    )

    # obstacle 3
    x_ratio_plane = 2/3
    y_pos = -12
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_3_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=3, width=7,
        height=height_hill+3,
        color=color
      )
    )

    # obstacle 4
    x_ratio_plane = 2/3
    y_pos = -25
    color = get_random_color()
    obstacles.append(box_xml(
        name=f"obstacle_4_{hill_id}",
        x=offset[0]+length_hill_up+length_plane*x_ratio_plane,
        y=offset[1]+y_pos,
        yaw=0,
        length=3, width=7,
        height=height_hill+3,
        color=color
      )
    )

    return obstacles


def generate_obstacles():
    obstacles = []
    obstacles.extend(generate_obstacles_hill_1(offset=(5, 0)))
    obstacles.extend(generate_obstacles_hill_2(offset=(90, 0)))
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

#!/usr/bin/env python3
from pathlib import Path

import yaml


DEFAULT_GAP_HILLS = 50.0
WAYPOINT_ALTITUDE_ABOVE_GROUND = 6.0


class IndentedSafeDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def load_crete_params(params_path: Path) -> dict:
    params = yaml.safe_load(params_path.read_text())
    if params is None:
        return {}
    if not isinstance(params, dict):
        raise ValueError(f"{params_path} must contain a YAML mapping")
    return params


def hill_length(hill_config: dict) -> float:
    return float(
        hill_config["length_ramp_up"]
        + hill_config["length_plane"]
        + hill_config["length_ramp_down"]
    )


def hill_center_x(hill_config: dict, hill_offset_x: float) -> float:
    return hill_offset_x + hill_length(hill_config) / 2.0


def waypoint(name: str, x: float, y: float, z: float, yaw_deg: float = 0.0) -> dict:
    return {
        "name": name,
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "yaw_deg": float(yaw_deg),
    }


def generate_waypoints(crete_params: dict) -> dict:
    hills = crete_params.get("hills", [])
    if not isinstance(hills, list):
        raise ValueError("crete_params.yaml field 'hills' must be a list")
    if len(hills) < 2:
        raise ValueError("crete_params.yaml must define at least two hills")

    hill_1 = hills[0]
    hill_2 = hills[1]
    if not isinstance(hill_1, dict) or not isinstance(hill_2, dict):
        raise ValueError("The first two hill entries must be mappings")
    
    offset_x = crete_params.get("offset_x", 5)
    offset_y = crete_params.get("offset_y", 0)
    gap_hills = crete_params.get("gap_hills", DEFAULT_GAP_HILLS)

    hill_1_offset_x = offset_x
    hill_1_length = hill_length(hill_1)
    hill_1_width = float(hill_1["width"])
    hill_1_y = 3.0 / 4.0 * hill_1_width / 2 + offset_y
    hill_1_z = float(hill_1["height"]) + WAYPOINT_ALTITUDE_ABOVE_GROUND
    hill_1_end_x = hill_1_offset_x + hill_1_length

    hill_1_center_x = hill_center_x(hill_1, hill_1_offset_x)
    hill_2_offset_x = hill_1_length + gap_hills

    midpoint_between_hills_x = (hill_1_end_x + hill_2_offset_x) / 2.0

    hill_2_center_x = hill_center_x(hill_2, hill_2_offset_x)
    hill_2_width = float(hill_2["width"])
    hill_2_z = float(hill_2["height"]) + WAYPOINT_ALTITUDE_ABOVE_GROUND
    hill_2_y = 3.0 / 4.0 * hill_2_width / 2 + offset_y

    return {
        "frame_id": "world",
        "inter_waypoint_delay": 1.0,
        "retry_delay": 2.0,
        "max_retries": 0,
        "waypoints": [
            waypoint("waypoint_1", 0.0, hill_1_y, WAYPOINT_ALTITUDE_ABOVE_GROUND, 0.0),

            waypoint("waypoint_2", midpoint_between_hills_x, hill_1_y, WAYPOINT_ALTITUDE_ABOVE_GROUND, 0.0),

            waypoint("waypoint_3", hill_2_center_x, hill_2_y, hill_2_z, 0.0),
            waypoint("waypoint_4", hill_2_center_x, -hill_2_y, hill_2_z, -90.0),

            waypoint("waypoint_5", hill_1_center_x, -hill_1_y, hill_1_z, 180),
            waypoint("waypoint_6", hill_1_center_x, hill_1_y, hill_1_z, 90.0),

            waypoint("waypoint_7", 0.0, 0.0, WAYPOINT_ALTITUDE_ABOVE_GROUND, 0.0),

            # waypoint("waypoint_1", -10.0, 0.0, 3.0, 0),
            # waypoint("waypoint_2", -10, 10, 3.0, 0),
            # waypoint("waypoint_3", -10, 20, 3.0, 0.0),

            # waypoint("waypoint_1", hill_2_center_x, hill_2_y, hill_2_z, 0.0),
            # waypoint("waypoint_2", 0.0, 0.0, 3.0, 0.0),
            # waypoint("waypoint_3", hill_2_center_x, -hill_2_y, hill_2_z, -90.0),
            # waypoint("waypoint_4", hill_2_center_x, hill_2_y, hill_2_z, 0.0),
        ],
    }


def main():
    waypoint_dir = Path(__file__).resolve().parent
    robot_bringup_dir = waypoint_dir.parents[2]
    crete_params_path = robot_bringup_dir / "gz" / "worlds" / "crete_params.yaml"
    waypoints_path = waypoint_dir / "waypoints.yaml"

    crete_params = load_crete_params(crete_params_path)
    waypoints = generate_waypoints(crete_params)
    waypoints_path.write_text(yaml.dump(waypoints, Dumper=IndentedSafeDumper, sort_keys=False))
    print(f"Wrote waypoints to {waypoints_path}")


if __name__ == "__main__":
    main()

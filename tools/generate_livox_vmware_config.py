#!/usr/bin/env python3
"""Generate a local Livox MID360 configuration for a VMware guest.

The driver config contains the *guest* address as seen by the lidar.  Keeping
that address in .echo_drone.env lets each vehicle/VM use its own IP without
changing a tracked JSON file.
"""

import argparse
import ipaddress
import json
import os
from pathlib import Path


def ipv4_from_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise SystemExit(f"ERROR: {name} is required; set it in .echo_drone.env")
    try:
        address = ipaddress.ip_address(value)
    except ValueError as error:
        raise SystemExit(f"ERROR: {name} must be an IPv4 address, got {value!r}") from error
    if address.version != 4:
        raise SystemExit(f"ERROR: {name} must be IPv4, got {value!r}")
    return str(address)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="generated JSON path")
    args = parser.parse_args()

    host_ip = ipv4_from_env("LIVOX_HOST_IP")
    lidar_ip = ipv4_from_env("LIVOX_LIDAR_IP")
    config = {
        "lidar_summary_info": {"lidar_type": 8},
        "Mid360s": {
            "lidar_net_info": {
                "cmd_data_port": 56100,
                "push_msg_port": 56200,
                "point_data_port": 56300,
                "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            "host_net_info": [{
                "host_ip": host_ip,
                "cmd_data_port": 56101,
                "push_msg_port": 56201,
                "point_data_port": 56301,
                "imu_data_port": 56401,
                "log_data_port": 56501,
            }],
        },
        "lidar_configs": [{
            "ip": lidar_ip,
            "pcl_data_type": 1,
            "pattern_mode": 0,
            "extrinsic_parameter": {
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                "x": 0, "y": 0, "z": 0,
            },
        }],
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2) + "\n", encoding="utf-8")
    print(f"Livox VMware config: {output}")
    print(f"  guest host IP: {host_ip}")
    print(f"  lidar IP:      {lidar_ip}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Generate PX4-native actuators.json for QGC Actuator Setup page.

Reads the PWM parameter definitions from rover_actuator_params.c
and produces a PX4-compatible actuators.json that QGC 5.0.8 can
consume via the Component Metadata FTP flow.

Output: build/generated_metadata/actuators.json
"""

import json
import sys
import os

NUM_OUTPUTS = 6

def generate_actuators_json():
    """Generate the actuators.json structure."""

    outputs = []
    for i in range(1, NUM_OUTPUTS + 1):
        output = {
            "label": f"PWM S{i}",
            "param": f"PWM_S{i}_FUNC",
            "min": 1000,
            "max": 2000,
            "default": 1500,
            "reversed": False,
            "count": 1,
            "groups": [
                {
                    "label": f"Motor Output S{i}",
                    "params": {
                        "min": f"PWM_S{i}_MIN",
                        "max": f"PWM_S{i}_MAX",
                        "center": f"PWM_S{i}_CENT",
                        "reverse": f"PWM_S{i}_REV"
                    },
                    "min": 1000,
                    "max": 2000,
                    "default": 1500
                }
            ]
        }
        outputs.append(output)

    actuator_types = [
        {
            "label": "PWM",
            "type": "pwm",
            "outputs": outputs,
            "functions": [
                {
                    "index": 0,
                    "label": "Disabled",
                    "default": True
                },
                {
                    "index": 101,
                    "label": "Motor right"
                },
                {
                    "index": 102,
                    "label": "Motor left"
                }
            ]
        }
    ]

    config = {
        "actuators": [
            {
                "label": "PWM Outputs",
                "actuatorType": "PWM",
                "supportedTypes": ["pwm"],
                "count": NUM_OUTPUTS,
                "groups": [
                    {
                        "label": "PWM Output",
                        "actuatorType": "pwm",
                        "startIndex": 0,
                        "count": NUM_OUTPUTS,
                        "parameters": [
                            {
                                "label": "Function",
                                "name": "FUNC",
                                "category": "Identity",
                                "indexAsName": True
                            },
                            {
                                "label": "Minimum PWM (us)",
                                "name": "MIN",
                                "category": "Configuration",
                                "indexAsName": True
                            },
                            {
                                "label": "Center PWM (us)",
                                "name": "CENT",
                                "category": "Configuration",
                                "indexAsName": True
                            },
                            {
                                "label": "Maximum PWM (us)",
                                "name": "MAX",
                                "category": "Configuration",
                                "indexAsName": True
                            },
                            {
                                "label": "Reverse",
                                "name": "REV",
                                "category": "Configuration",
                                "indexAsName": True,
                                "advanced": True
                            }
                        ]
                    }
                ],
                "items": [
                    {
                        "label": "Motor right",
                        "function": 101,
                        "note": "Right side motor output"
                    },
                    {
                        "label": "Motor left",
                        "function": 102,
                        "note": "Left side motor output"
                    }
                ]
            }
        ],
        "mixer": {
            "actuatorTypes": actuator_types
        }
    }

    return config


def main():
    output_dir = sys.argv[1] if len(sys.argv) > 1 else "build/generated_metadata"
    os.makedirs(output_dir, exist_ok=True)

    config = generate_actuators_json()
    output_path = os.path.join(output_dir, "actuators.json")

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    print(f"Generated {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

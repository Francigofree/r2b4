alba@raspberrypi:~/project_r2b4 $ cd ~/project_r2b4
./r rt start
./r s --json
runtime: STARTED (PID 17299)
capture: ARMED (ALAP, native MCAP 8+2 bounded ring, 10 Hz)
capture file: runtime/captures/v3_20261010_232038_17262_capture.mcap
runtime log: /tmp/r2b4-runtime.26i3t0q4.log
{
  "capture_hz": 10,
  "capture_mode": "alap",
  "fault_layer": null,
  "pid": 17299,
  "ready": true,
  "runtime": "RUNNING",
  "safety": "STOP",
  "safety_reason": "NOT_ACTIVE",
  "state": "RUNNING",
  "tick": 107
}
alba@raspberrypi:~/project_r2b4 $ 

------------------------------


ndoor state requires a corresponding perception owner and fails explicitly when\nunavailable. No LLM is called while this program runs.",
      "kind": "action",
      "name": "skill.monitor_region",
      "owner": "host",
      "parameters": {
        "duration_hours": {
          "default": "8",
          "required": false
        },
        "fps": {
          "default": "5",
          "required": false
        },
        "object_kind": {
          "default": "'person'",
          "required": false
        },
        "output_dir": {
          "default": "None",
          "required": false
        },
        "post_s": {
          "default": "3",
          "required": false
        },
        "pre_s": {
          "default": "2",
          "required": false
        },
        "region": {
          "default": "None",
          "required": false
        }
      },
      "ready": true,
      "result": "Python return value",
      "skill_name": "monitor_region",
      "source_hash": "fa91fa751888005159d1c48ccb63677a3853c78b489b6de7f92d84b6f7d4ff0b",
      "source_path": "/home/alba/project_r2b4/robot_skills/monitor_region.py",
      "supported": true
    },
    "skill.revoke": {
      "adapter": "public_robot",
      "available": true,
      "description": "Revoke an invocation immediately without waiting for worker cleanup.",
      "kind": "action",
      "name": "skill.revoke",
      "owner": "host",
      "parameters": {
        "reason": {
          "type": "string"
        },
        "run_id": {
          "type": "string"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.run": {
      "adapter": "public_robot",
      "available": true,
      "description": "Start a saved Python skill in a separate interpreter; returns a run_id.",
      "kind": "action",
      "name": "skill.run",
      "owner": "host",
      "parameters": {
        "name": {
          "required": true,
          "type": "string"
        },
        "parameters": {
          "type": "object"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.source": {
      "adapter": "public_robot",
      "available": true,
      "description": "Read the saved source and its hash for a targeted repair.",
      "kind": "action",
      "name": "skill.source",
      "owner": "host",
      "parameters": {
        "name": {
          "required": true,
          "type": "string"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.status": {
      "adapter": "public_robot",
      "available": true,
      "description": "Read a run's state, returned result or error; success is program completion.",
      "kind": "action",
      "name": "skill.status",
      "owner": "host",
      "parameters": {
        "run_id": {
          "type": "string"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.stop": {
      "adapter": "public_robot",
      "available": true,
      "description": "Revoke and stop a skill and its own canonical motion.",
      "kind": "action",
      "name": "skill.stop",
      "owner": "host",
      "parameters": {
        "reason": {
          "type": "string"
        },
        "run_id": {
          "type": "string"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.test": {
      "adapter": "public_robot",
      "available": true,
      "description": "Run the skill's optional saved pytest through the canonical launcher; no physical-goal proof.",
      "kind": "action",
      "name": "skill.test",
      "owner": "host",
      "parameters": {
        "name": {
          "required": true,
          "type": "string"
        },
        "timeout_s": {
          "type": "number"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "skill.update": {
      "adapter": "public_robot",
      "available": true,
      "description": "Save a new source version; the running worker keeps its loaded version.",
      "kind": "action",
      "name": "skill.update",
      "owner": "host",
      "parameters": {
        "description": {
          "type": "string"
        },
        "name": {
          "required": true,
          "type": "string"
        },
        "source": {
          "required": true,
          "type": "string"
        }
      },
      "ready": true,
      "result": {
        "description": "Saved descriptor or invocation state/result."
      },
      "supported": true
    },
    "source.read": {
      "adapter": "public_robot",
      "available": true,
      "description": "Read a bounded line range from current R2B4 Python/shell source ortests.",
      "kind": "action",
      "name": "source.read",
      "owner": "host",
      "parameters": {
        "end_line": "optional integer; max 240 lines",
        "path": "required repository-relative path",
        "start_line": "optional positive integer"
      },
      "ready": true,
      "result": {
        "description": "Existing owning tool result."
      },
      "supported": true
    },
    "source.search": {
      "adapter": "public_robot",
      "available": true,
      "description": "Search current R2B4 Python/shell source and tests. Runtime, old backups, hidden and secret files are excluded.",
      "kind": "action",
      "name": "source.search",
      "owner": "host",
      "parameters": {
        "limit": "optional integer 1..50",
        "query": "required string"
      },
      "ready": true,
      "result": {
        "description": "Existing owning tool result."
      },
      "supported": true
    },
    "spatial.load_atlas": {
      "adapter": "public_robot",
      "available": true,
      "description": "spatial load_atlas",
      "kind": "action",
      "name": "spatial.load_atlas",
      "owner": "host",
      "parameters": {},
      "ready": true,
      "reason": "HOST_KNOWLEDGE_UPDATE",
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "spatial.query": {
      "adapter": "public_robot",
      "available": true,
      "description": "spatial query",
      "kind": "read",
      "name": "spatial.query",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "spatial.snapshot": {
      "adapter": "public_robot",
      "available": true,
      "description": "spatial snapshot",
      "kind": "read",
      "name": "spatial.snapshot",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "spatial.teach_place": {
      "adapter": "public_robot",
      "available": true,
      "description": "spatial teach_place",
      "kind": "action",
      "name": "spatial.teach_place",
      "owner": "host",
      "parameters": {},
      "ready": true,
      "reason": "HOST_KNOWLEDGE_UPDATE",
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "system.status": {
      "adapter": "system",
      "available": true,
      "description": "system status",
      "kind": "read",
      "name": "system.status",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "v3.command.backward": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Drive backward at a bounded linear speed.",
      "kind": "action",
      "name": "v3.command.backward",
      "parameters": {
        "speed_mps": {
          "default": 0.15,
          "description": "Backward speed magnitude in metres per second.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.explore": {
      "adapter": "v3_control",
      "available": true,
      "behavior": "room_cruise",
      "completion_required": false,
      "description": "Start autonomous room exploration using the canonical navigation stack.",
      "execution_owner": "behavior_system",
      "kind": "action",
      "name": "v3.command.explore",
      "parameters": {
        "explore_local_goal_distance_weight": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.0,
          "required": false,
          "type": "number"
        },
        "explore_local_goal_max_distance_m": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "explore_local_goal_min_distance_m": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "local_goal_clearance_weight": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.0,
          "required": false,
          "type": "number"
        },
        "local_goal_forward_weight": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.0,
          "required": false,
          "type": "number"
        },
        "local_goal_novelty_weight": {
          "default": null,
          "description": "EXPLORE goal preference; omitted values use the configured RoomCruise profile.",
          "maximum": null,
          "minimum": 0.0,
          "required": false,
          "type": "number"
        },
        "max_omega_rad_s": {
          "default": null,
          "description": "Requested angular cap; omitted values use behavior.roomcruise config.",
          "maximum": null,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "max_v_mps": {
          "default": null,
          "description": "Requested linear cap; omitted values use behavior.roomcruise config.",
          "maximum": null,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "CANONICAL_V3_EXECUTION",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.face_person": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Rotate in place toward the currently tracked person.",
      "kind": "action",
      "name": "v3.command.face_person",
      "parameters": {
        "max_omega_rad_s": {
          "default": 0.5,
          "description": "Maximum turning speed.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [
        "person_target"
      ],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.follow_person": {
      "adapter": "v3_control",
      "available": true,
      "behavior": "follow_person",
      "completion_required": false,
      "description": "Follow the currently tracked person using the canonical navigationstack.",
      "execution_owner": "behavior_system",
      "kind": "action",
      "name": "v3.command.follow_person",
      "parameters": {
        "expected_runtime_pid": {
          "default": null,
          "description": "Runtime identity of the bound person track; a restart rejects the request.",
          "maximum": null,
          "minimum": 1,
          "required": false,
          "type": "number"
        },
        "max_omega_rad_s": {
          "default": 0.3,
          "description": "Maximum following angular speed.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "max_v_mps": {
          "default": 0.25,
          "description": "Maximum following linear speed.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "target_track_id": {
          "default": null,
          "description": "Optional exact V3 person track; requires its expected_runtime_pid and never selects another person.",
          "maxLength": 256,
          "maximum": null,
          "minLength": 1,
          "minimum": null,
          "required": false,
          "type": "string"
        }
      },
      "ready": true,
      "reason": "CANONICAL_V3_EXECUTION",
      "requirements": [
        "person_target"
      ],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.forward": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Drive forward at a bounded linear speed.",
      "kind": "action",
      "name": "v3.command.forward",
      "parameters": {
        "speed_mps": {
          "default": 0.15,
          "description": "Forward speed in metres per second.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.move_relative": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": true,
      "description": "Move to a relative local pose using closed-loop navigation; blocksuntil completion or failure and STOP. Negative forward_m requests a goal behind the robot.",
      "kind": "action",
      "name": "v3.command.move_relative",
      "parameters": {
        "final_yaw_rad": {
          "default": null,
          "description": "Optional final heading offset in radians relative to the starting heading.",
          "maximum": null,
          "minimum": null,
          "required": false,
          "type": "number"
        },
        "forward_m": {
          "default": null,
          "description": "Forward distance in metres relative to the starting pose; negative means behind.",
          "maximum": null,
          "minimum": null,
          "required": true,
          "type": "number"
        },
        "left_m": {
          "default": 0.0,
          "description": "Leftward relative goal offset in metres, not sideways driving.",
          "maximum": null,
          "minimum": null,
          "required": false,
          "type": "number"
        },
        "max_omega_rad_s": {
          "default": 0.6,
          "description": "Maximum turning speed.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "max_v_mps": {
          "default": 0.2,
          "description": "Maximum linear speed.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.navigate": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Navigate to a pose in the current localization frame using local closed-loop control.",
      "kind": "action",
      "name": "v3.command.navigate",
      "parameters": {
        "frame_id": {
          "default": null,
          "description": "R2B4_BOOT_ROBOT_MAP (default) or R2B4_ODOM_LOCAL.",
          "maxLength": 256,
          "maximum": null,
          "minLength": 1,
          "minimum": null,
          "required": false,
          "type": "string"
        },
        "max_omega_rad_s": {
          "default": 0.6,
          "description": "Maximum turning speed.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "max_v_mps": {
          "default": 0.2,
          "description": "Maximum linear speed.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "x_m": {
          "default": null,
          "description": "Target x in metres in the current localization frame.",
          "maximum": null,
          "minimum": null,
          "required": true,
          "type": "number"
        },
        "y_m": {
          "default": null,
          "description": "Target y in metres in the current localization frame.",
          "maximum": null,
          "minimum": null,
          "required": true,
          "type": "number"
        },
        "yaw_rad": {
          "default": null,
          "description": "Optional final heading in radians; omitted means any heading.",
          "maximum": null,
          "minimum": null,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": false
    },
    "v3.command.stop": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Stop current robot motion through the canonical fail-safe command path.",
      "kind": "action",
      "name": "v3.command.stop",
      "parameters": {},
      "ready": true,
      "reason": "STOP_IS_SAFE_NOOP_WHEN_IDLE",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": false,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.teleop": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Command bounded linear and angular robot velocity.",
      "kind": "action",
      "name": "v3.command.teleop",
      "parameters": {
        "max_omega_rad_s": {
          "default": 1.2,
          "description": "Absolute angular speed limit.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "max_v_mps": {
          "default": 0.5,
          "description": "Absolute linear speed limit.",
          "maximum": 0.5,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        },
        "omega_rad_s": {
          "default": null,
          "description": "Requested angular velocity in radians per second.",
          "maximum": 1.2,
          "minimum": -1.2,
          "required": true,
          "type": "number"
        },
        "v_mps": {
          "default": null,
          "description": "Requested linear velocity in metres per second.",
          "maximum": 0.5,
          "minimum": -0.5,
          "required": true,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.turn_by": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": true,
      "description": "Turn in place by a relative angle using closed-loop heading control; blocks until completion or failure and STOP.",
      "kind": "action",
      "name": "v3.command.turn_by",
      "parameters": {
        "angle_deg": {
          "default": null,
          "description": "Relative angle in degrees; positive turns left, negative right. Pose goals use the shortest turn.",
          "maximum": 180.0,
          "minimum": -180.0,
          "required": true,
          "type": "number"
        },
        "max_omega_rad_s": {
          "default": 0.6,
          "description": "Maximum turning speed.",
          "maximum": 1.2,
          "minimum": 0.01,
          "required": false,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": true
    },
    "v3.command.wheels": {
      "adapter": "v3_control",
      "available": true,
      "completion_required": false,
      "description": "Command bounded left and right wheel target speeds.",
      "kind": "action",
      "name": "v3.command.wheels",
      "parameters": {
        "left_mps": {
          "default": null,
          "description": "Left wheel target speed in metres per second.",
          "maximum": 0.5,
          "minimum": -0.5,
          "required": true,
          "type": "number"
        },
        "right_mps": {
          "default": null,
          "description": "Right wheel target speed in metres per second.",
          "maximum": 0.5,
          "minimum": -0.5,
          "required": true,
          "type": "number"
        }
      },
      "ready": true,
      "reason": "RUNTIME_READY",
      "requirements": [],
      "result": {
        "description": "Owner's public state or operation result."
      },
      "schema": "R2B4_ACTION_DESCRIPTOR_V1",
      "session_watchdog": true,
      "supported": true,
      "voice_exposed": false
    },
    "v3.health": {
      "adapter": "v3_control",
      "available": true,
      "description": "v3 health",
      "kind": "read",
      "name": "v3.health",
      "parameters": {},
      "ready": true,
      "reason": null,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "v3.pose": {
      "adapter": "v3_control",
      "available": true,
      "description": "v3 pose",
      "kind": "read",
      "name": "v3.pose",
      "parameters": {},
      "ready": true,
      "reason": null,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "v3.safety": {
      "adapter": "v3_control",
      "available": true,
      "description": "v3 safety",
      "kind": "read",
      "name": "v3.safety",
      "parameters": {},
      "ready": true,
      "reason": null,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "v3.status": {
      "adapter": "v3_control",
      "available": true,
      "description": "v3 status",
      "kind": "read",
      "name": "v3.status",
      "parameters": {},
      "ready": true,
      "reason": null,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "vision.observe": {
      "adapter": "camera",
      "available": true,
      "description": "vision observe",
      "kind": "action",
      "name": "vision.observe",
      "parameters": {},
      "ready": false,
      "reason": "CALIBRATED_DEMAND_DRIVEN",
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "world.history": {
      "adapter": "public_robot",
      "available": true,
      "description": "world history",
      "kind": "read",
      "name": "world.history",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "world.observe": {
      "adapter": "public_robot",
      "available": true,
      "description": "world observe",
      "kind": "action",
      "name": "world.observe",
      "owner": "host",
      "parameters": {},
      "ready": true,
      "reason": "V3_INDEPENDENT_PUBLIC_STATE",
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "world.query": {
      "adapter": "public_robot",
      "available": true,
      "description": "world query",
      "kind": "read",
      "name": "world.query",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    },
    "world.snapshot": {
      "adapter": "public_robot",
      "available": true,
      "description": "world snapshot",
      "kind": "read",
      "name": "world.snapshot",
      "parameters": {},
      "ready": true,
      "result": {
        "description": "Owner's public state or operation result."
      },
      "supported": true
    }
  },
  "schema": "R2B4_ROBOT_INTERFACE_V2"
}
[
  {
    "availability": {
      "available": true,
      "ready": true,
      "supported": true
    },
    "description": "",
    "name": "live_probe_261010",
    "parameters": {
      "count": {
        "default": "8",
        "required": false
      },
      "dt_s": {
        "default": "0.25",
        "required": false
      }
    },
    "result": "Python return value",
    "source_hash": "d30cca903bcd9a1350933cc54508ff352e45ebc25ba11e767d6fbe7a1696fd8e",
    "source_path": "/home/alba/project_r2b4/robot_skills/live_probe_261010.py"
  },
  {
    "availability": {
      "available": true,
      "ready": true,
      "supported": true
    },
    "description": "Monitor an image-space region locally and record qualified detector events.\n\nThe installed detector recognizes people. A named room, mouse recognition or\ndoor state requires a corresponding perception owner and fails explicitly when\nunavailable. No LLM is called while this program runs.",
    "name": "monitor_region",
    "parameters": {
      "duration_hours": {
        "default": "8",
        "required": false
      },
      "fps": {
        "default": "5",
        "required": false
      },
      "object_kind": {
        "default": "'person'",
        "required": false
      },
      "output_dir": {
        "default": "None",
        "required": false
      },
      "post_s": {
        "default": "3",
        "required": false
      },
      "pre_s": {
        "default": "2",
        "required": false
      },
      "region": {
        "default": "None",
        "required": false
      }
    },
    "result": "Python return value",
    "source_hash": "fa91fa751888005159d1c48ccb63677a3853c78b489b6de7f92d84b6f7d4ff0b",
    "source_path": "/home/alba/project_r2b4/robot_skills/monitor_region.py"
  }
]
alba@raspberrypi:~/project_r2b4 $ 

------------------------------

alba@raspberrypi:~/project_r2b4 $ ./r execute skill.run --parameters '{"name":"live_probe_261010","parameters":{"count":8}}' --json
{
  "error": null,
  "finalizing": false,
  "finished_ns": null,
  "generation": 11,
  "name": "live_probe_261010",
  "output_dropped_bytes": 0,
  "parameters": {
    "count": 8
  },
  "pid": null,
  "result": null,
  "returncode": null,
  "revision": 1,
  "run_id": "d14bd5567e8c4a7780a0eaf6431a85fb",
  "source_hash": "d30cca903bcd9a1350933cc54508ff352e45ebc25ba11e767d6fbe7a1696fd8e",
  "source_path": "/home/alba/project_r2b4/robot_skills/live_probe_261010.py",
  "started_ns": 15745366135337,
  "state": "STARTING",
  "stderr": "",
  "stdout": ""
}
alba@raspberrypi:~/project_r2b4 $ 

----------------------------------

alba@raspberrypi:~/project_r2b4 $ ./r execute skill.status --parameters '{"run_id":"d14bd5567e8c4a7780a0eaf6431a85fb"}' --json
{
  "error": null,
  "finalizing": false,
  "finished_ns": 15747810090597,
  "generation": 11,
  "name": "live_probe_261010",
  "output_dropped_bytes": 0,
  "parameters": {
    "count": 8
  },
  "pid": 17624,
  "result": {
    "checks": 8,
    "passed": 8,
    "report_recorded": true,
    "revision": "A",
    "status": "COMPLETED"
  },
  "returncode": 0,
  "revision": 3,
  "run_id": "d14bd5567e8c4a7780a0eaf6431a85fb",
  "source_hash": "d30cca903bcd9a1350933cc54508ff352e45ebc25ba11e767d6fbe7a1696fd8e",
  "source_path": "/home/alba/project_r2b4/robot_skills/live_probe_261010.py",
  "started_ns": 15745366135337,
  "state": "SUCCEEDED",
  "stderr": "",
  "stdout": ""
}
alba@raspberrypi:~/project_r2b4 $ 


-----------------------------------


alba@raspberrypi:~/project_r2b4 $ ./r execute skill.run --parameters '{"name":"monitor_region","parameters":{"object_kind":"person","duration_hours":0.01,"fps":5}}' --json
{
  "error": null,
  "finalizing": false,
  "finished_ns": null,
  "generation": 12,
  "name": "monitor_region",
  "output_dropped_bytes": 0,
  "parameters": {
    "duration_hours": 0.01,
    "fps": 5,
    "object_kind": "person"
  },
  "pid": null,
  "result": null,
  "returncode": null,
  "revision": 1,
  "run_id": "d88148d3371d449486b0e36a6fc0eae2",
  "source_hash": "fa91fa751888005159d1c48ccb63677a3853c78b489b6de7f92d84b6f7d4ff0b",
  "source_path": "/home/alba/project_r2b4/robot_skills/monitor_region.py",
  "started_ns": 15889601393617,
  "state": "STARTING",
  "stderr": "",
  "stdout": ""
}
alba@raspberrypi:~/project_r2b4 $ 


--------------------------------

alba@raspberrypi:~/project_r2b4 $ ./r execute skill.run --parameters '{"name":"monitor_region","parameters":{"object_kind":"mouse","duration_hours":0.001}}' --json
{
  "error": null,
  "finalizing": false,
  "finished_ns": null,
  "generation": 13,
  "name": "monitor_region",
  "output_dropped_bytes": 0,
  "parameters": {
    "duration_hours": 0.001,
    "object_kind": "mouse"
  },
  "pid": null,
  "result": null,
  "returncode": null,
  "revision": 1,
  "run_id": "60a989a871924465b70616a57845be82",
  "source_hash": "fa91fa751888005159d1c48ccb63677a3853c78b489b6de7f92d84b6f7d4ff0b",
  "source_path": "/home/alba/project_r2b4/robot_skills/monitor_region.py",
  "started_ns": 15938217737909,
  "state": "STARTING",
  "stderr": "",
  "stdout": ""
}
alba@raspberrypi:~/project_r2b4 $ 


------------------------------


alba@raspberrypi:~/project_r2b4 $ ./r execute v3.command.move_relative --parameters '{"forward_m":0.2,"max_v_mps":0.15}' --json

./r execute v3.command.turn_by --parameters '{"angle_deg":30,"max_omega_rad_s":0.3}' --json
navigate: STARTED
command: NAVIGATE mission acknowledged
capture: ALAP 10 Hz armed for STOP/FAULT -> runtime/captures/v3_20261010_232038_17262_capture.mcap
capture: TRIGGERED (movement-stop)
capture file: runtime/captures/v3_20261010_232038_17262_capture.mcap
{
  "angle_executed_rad": null,
  "angle_remaining_rad": null,
  "angle_requested_rad": null,
  "command_id": "operator-navigate-1791667560585624852-18900",
  "completion_status_monotonic_ns": 15996831940817,
  "distance_executed_m": 0.20064988896443595,
  "distance_remaining_m": 0.019239152192704854,
  "distance_requested_m": 0.2,
  "elapsed_s": 7.869779161999759,
  "final_pose": {
    "__type__": "Pose2D",
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.19863785956693372,
    "y_m": -0.028343935649581084,
    "yaw_rad": -0.7299105280865567
  },
  "frame_provenance": {
    "config_snapshot_id": "eed651d2705313930fd2a896e902842c9c800b949a372e50bb52803a8da472bf",
    "frame_id": "R2B4_ODOM_LOCAL",
    "localization_generation": 0,
    "pose_status_monotonic_ns": 15991391992035,
    "runtime_pid": 17299
  },
  "goal_tolerance_m": 0.020000000000000004,
  "mission_id": "mission-operator-navigate-1791667560585624852-18900",
  "navigation_reason": null,
  "progress": 1.0,
  "reason": "COMPLETE",
  "requested": {
    "forward_m": 0.2,
    "max_v_mps": 0.15
  },
  "runtime_pid": 17299,
  "safety_reason": null,
  "start_pose": {
    "__type__": "Pose2D",
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": -0.2377566537307942
  },
  "status": "COMPLETED",
  "stop": {
    "status": "STOPPED"
  },
  "target_pose": {
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.19437375590540126,
    "y_m": -0.04710459654033239
  },
  "yaw_tolerance_rad": 0.1
}
capture: previous bounded slot used -> re-arm requires runtime restart
runtime: existing instance -> STOP
capture: waiting for 2 s post-event tail
runtime: STOPPED (PID 17299)
capture: runtime/captures/v3_20261010_232038_17262_capture.mcap
capture integrity failed after runtime shutdown: CAPTURE_INCOMPLETE: required evidence is incomplete
runtime: waiting 5 s
runtime: STARTED (PID 19070)
capture: ARMED (ALAP, native MCAP 8+2 bounded ring, 10 Hz)
capture file: runtime/captures/v3_20261010_232627_18978_capture.mcap
runtime log: /tmp/r2b4-runtime.l1kzwqzn.log
{
  "angle_executed_rad": 0.0,
  "angle_remaining_rad": 0.5235987755982987,
  "angle_requested_rad": 0.5235987755982987,
  "command_id": null,
  "completion_status_monotonic_ns": 16024278099708,
  "distance_executed_m": 0.0,
  "distance_remaining_m": 0.0,
  "distance_requested_m": 0.0,
  "elapsed_s": 24.538195353001356,
  "final_pose": {
    "__type__": "Pose2D",
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": -0.7581273938350369
  },
  "frame_provenance": {
    "config_snapshot_id": "eed651d2705313930fd2a896e902842c9c800b949a372e50bb52803a8da472bf",
    "frame_id": "R2B4_ODOM_LOCAL",
    "localization_generation": 0,
    "pose_status_monotonic_ns": 16024278099708,
    "runtime_pid": 19070
  },
  "goal_tolerance_m": 0.08,
  "mission_id": null,
  "navigation_reason": "COMMAND_STOP",
  "progress": 0.0,
  "reason": "ANGULAR_LIMIT_UNREALIZABLE",
  "requested": {
    "angle_deg": 30,
    "max_omega_rad_s": 0.3
  },
  "runtime_pid": 19070,
  "safety_reason": "NOT_ACTIVE",
  "start_pose": {
    "__type__": "Pose2D",
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": -0.7581273938350369
  },
  "status": "INTERRUPTED",
  "stop": {
    "status": "STOPPED"
  },
  "target_pose": {
    "frame_id": "R2B4_ODOM_LOCAL",
    "x_m": 0.0,
    "y_m": 0.0,
    "yaw_rad": -0.23452861823673832
  },
  "yaw_tolerance_rad": 0.052359877559829876
}
alba@raspberrypi:~/project_r2b4 $ 
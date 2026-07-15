from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from typing import Any


DEFAULT_CARLA_VEHICLE_MODEL = "vehicle.tesla.model3"


def build_openscenario_xml(scenario_ir: dict[str, Any], *, road_file: str = "road.xodr") -> str:
    yaw_is_degrees = _yaw_is_degrees(scenario_ir)
    root = ET.Element("OpenSCENARIO")
    ET.SubElement(
        root,
        "FileHeader",
        {
            "revMajor": "1",
            "revMinor": "0",
            "date": "2026-07-08T00:00:00",
            "description": f"Generated from Scenario IR {scenario_ir['scenario_id']}",
            "author": "ClosedLoopBench",
        },
    )

    ET.SubElement(root, "ParameterDeclarations")
    ET.SubElement(root, "CatalogLocations")

    road_network = ET.SubElement(root, "RoadNetwork")
    ET.SubElement(road_network, "LogicFile", {"filepath": road_file})

    entities = ET.SubElement(root, "Entities")
    _add_vehicle_entity(entities, "ego", scenario_ir.get("ego", {}).get("dimensions"))
    for actor in scenario_ir.get("actors", []):
        actor_id = str(actor.get("actor_id"))
        if actor_id:
            _add_vehicle_entity(entities, _safe_name(actor_id), actor.get("dimensions"))

    storyboard = ET.SubElement(root, "Storyboard")
    init = ET.SubElement(storyboard, "Init")
    actions = ET.SubElement(init, "Actions")
    _add_init_action(
        actions,
        "ego",
        scenario_ir.get("ego", {}).get("initial_state"),
        yaw_is_degrees=yaw_is_degrees,
    )
    for actor in scenario_ir.get("actors", []):
        _add_init_action(
            actions,
            _safe_name(str(actor.get("actor_id"))),
            actor.get("initial_state"),
            yaw_is_degrees=yaw_is_degrees,
        )

    event_end = float(scenario_ir.get("windows", {}).get("event", {}).get("end_sec", 10.0))
    replay_actors = [
        (actor, actor.get("reference_trajectory") or [])
        for actor in scenario_ir.get("actors", [])
        if _is_replay_actor(actor) and len(actor.get("reference_trajectory") or []) >= 2
    ]
    if replay_actors:
        story = ET.SubElement(storyboard, "Story", {"name": "story"})
        act = ET.SubElement(story, "Act", {"name": "act"})
        for actor, trajectory in replay_actors:
            _add_replay_maneuver_group(
                act,
                actor,
                trajectory,
                yaw_is_degrees=yaw_is_degrees,
            )
        _add_simulation_time_trigger(act, "StartTrigger", "act_start", 0.0, rule="greaterThan")
        _add_simulation_time_trigger(act, "StopTrigger", "act_end", event_end, rule="greaterThan")
    stop = ET.SubElement(storyboard, "StopTrigger")
    condition_group = ET.SubElement(stop, "ConditionGroup")
    condition = ET.SubElement(
        condition_group,
        "Condition",
        {"name": "event_window_end", "delay": "0", "conditionEdge": "rising"},
    )
    by_value = ET.SubElement(condition, "ByValueCondition")
    sim_time = str(scenario_ir.get("windows", {}).get("event", {}).get("end_sec", 10.0))
    ET.SubElement(by_value, "SimulationTimeCondition", {"value": sim_time, "rule": "greaterThan"})

    _indent(root)
    return ET.tostring(root, encoding="unicode")


def _add_replay_maneuver_group(
    act: ET.Element,
    actor: dict[str, Any],
    trajectory: list[dict[str, Any]],
    *,
    yaw_is_degrees: bool,
) -> None:
    actor_name = _safe_name(str(actor.get("actor_id")))
    maneuver_group = ET.SubElement(
        act,
        "ManeuverGroup",
        {"maximumExecutionCount": "1", "name": f"{actor_name}_replay_group"},
    )
    actors = ET.SubElement(maneuver_group, "Actors", {"selectTriggeringEntities": "false"})
    ET.SubElement(actors, "EntityRef", {"entityRef": actor_name})
    maneuver = ET.SubElement(maneuver_group, "Maneuver", {"name": f"{actor_name}_replay_maneuver"})
    event = ET.SubElement(
        maneuver,
        "Event",
        {"maximumExecutionCount": "1", "name": f"{actor_name}_replay_event", "priority": "overwrite"},
    )
    action = ET.SubElement(event, "Action", {"name": f"{actor_name}_follow_trajectory"})
    private_action = ET.SubElement(action, "PrivateAction")
    routing_action = ET.SubElement(private_action, "RoutingAction")
    follow = ET.SubElement(routing_action, "FollowTrajectoryAction")
    trajectory_node = ET.SubElement(
        follow,
        "Trajectory",
        {"name": f"{actor_name}_reference_trajectory", "closed": "false"},
    )
    ET.SubElement(trajectory_node, "ParameterDeclarations")
    polyline = ET.SubElement(ET.SubElement(trajectory_node, "Shape"), "Polyline")
    for point in trajectory:
        vertex = ET.SubElement(polyline, "Vertex", {"time": _format_number(_trajectory_time(point))})
        position = ET.SubElement(vertex, "Position")
        ET.SubElement(
            position,
            "WorldPosition",
            {
                "x": _format_number(float(point.get("x", 0.0))),
                "y": _format_number(float(point.get("y", 0.0))),
                "z": _format_number(float(point.get("z", 0.0))),
                "h": _format_number(
                    _heading_radians(point.get("yaw", point.get("h", 0.0)), yaw_is_degrees)
                ),
            },
        )
    time_reference = ET.SubElement(follow, "TimeReference")
    ET.SubElement(
        time_reference,
        "Timing",
        {"domainAbsoluteRelative": "absolute", "scale": "1", "offset": "0"},
    )
    ET.SubElement(follow, "TrajectoryFollowingMode", {"followingMode": "position"})
    _add_simulation_time_trigger(
        event,
        "StartTrigger",
        f"{actor_name}_trajectory_start",
        _trajectory_time(trajectory[0]),
        rule="greaterThan",
    )


def _add_simulation_time_trigger(
    parent: ET.Element,
    tag: str,
    name: str,
    value: float,
    *,
    rule: str,
) -> ET.Element:
    trigger = ET.SubElement(parent, tag)
    condition_group = ET.SubElement(trigger, "ConditionGroup")
    condition = ET.SubElement(
        condition_group,
        "Condition",
        {"name": name, "delay": "0", "conditionEdge": "none"},
    )
    by_value = ET.SubElement(condition, "ByValueCondition")
    ET.SubElement(
        by_value,
        "SimulationTimeCondition",
        {"value": _format_number(value), "rule": rule},
    )
    return trigger


def _is_replay_actor(actor: dict[str, Any]) -> bool:
    closed_loop_level = actor.get("closed_loop_level")
    if closed_loop_level is not None:
        return str(closed_loop_level).lower() == "replay"
    policy = actor.get("policy", actor.get("policy_mode"))
    if policy is not None:
        return str(policy).lower() == "replay"
    return True


def _trajectory_time(point: dict[str, Any]) -> float:
    return float(point.get("t_sec", point.get("t", 0.0)))


def _yaw_is_degrees(scenario_ir: dict[str, Any]) -> bool:
    yaw_unit = scenario_ir.get("coordinate_frame", {}).get("units", {}).get("yaw", "radian")
    return str(yaw_unit).lower() in {"degree", "degrees", "deg"}


def _heading_radians(value: Any, yaw_is_degrees: bool) -> float:
    heading = float(value or 0.0)
    return math.radians(heading) if yaw_is_degrees else heading


def _add_vehicle_entity(parent, name: str, dimensions: dict[str, Any] | None = None) -> None:
    vehicle_dimensions = _vehicle_dimensions(dimensions)
    obj = ET.SubElement(parent, "ScenarioObject", {"name": name})
    vehicle = ET.SubElement(obj, "Vehicle", {"name": DEFAULT_CARLA_VEHICLE_MODEL, "vehicleCategory": "car"})
    ET.SubElement(vehicle, "ParameterDeclarations")
    bounding_box = ET.SubElement(vehicle, "BoundingBox")
    ET.SubElement(
        bounding_box,
        "Center",
        {"x": "0.0", "y": "0.0", "z": _format_number(vehicle_dimensions["height"] / 2.0)},
    )
    ET.SubElement(
        bounding_box,
        "Dimensions",
        {
            "width": _format_number(vehicle_dimensions["width"]),
            "length": _format_number(vehicle_dimensions["length"]),
            "height": _format_number(vehicle_dimensions["height"]),
        },
    )
    ET.SubElement(vehicle, "Performance", {"maxSpeed": "69.4", "maxAcceleration": "10.0", "maxDeceleration": "10.0"})
    axles = ET.SubElement(vehicle, "Axles")
    ET.SubElement(
        axles,
        "FrontAxle",
        {"maxSteering": "0.5", "wheelDiameter": "0.7", "trackWidth": "1.8", "positionX": "3.1", "positionZ": "0.35"},
    )
    ET.SubElement(
        axles,
        "RearAxle",
        {"maxSteering": "0.0", "wheelDiameter": "0.7", "trackWidth": "1.8", "positionX": "0.0", "positionZ": "0.35"},
    )
    ET.SubElement(vehicle, "Properties")


def _add_init_action(
    actions,
    entity_name: str,
    state: dict[str, Any] | None,
    *,
    yaw_is_degrees: bool,
) -> None:
    state = state or {}
    private = ET.SubElement(actions, "Private", {"entityRef": entity_name})
    private_action = ET.SubElement(private, "PrivateAction")
    teleport = ET.SubElement(private_action, "TeleportAction")
    position = ET.SubElement(teleport, "Position")
    ET.SubElement(
        position,
        "WorldPosition",
        {
            "x": str(state.get("x", 0.0)),
            "y": str(state.get("y", 0.0)),
            "z": str(state.get("z", 0.0)),
            "h": _format_number(_heading_radians(state.get("yaw", 0.0), yaw_is_degrees)),
        },
    )
    speed_action = ET.SubElement(ET.SubElement(private, "PrivateAction"), "LongitudinalAction")
    speed = ET.SubElement(speed_action, "SpeedAction")
    ET.SubElement(speed, "SpeedActionDynamics", {"dynamicsShape": "step", "value": "0", "dynamicsDimension": "time"})
    target = ET.SubElement(speed, "SpeedActionTarget")
    ET.SubElement(target, "AbsoluteTargetSpeed", {"value": str(state.get("speed_mps", 0.0))})


def _safe_name(value: str) -> str:
    return "actor_" + "".join(ch if ch.isalnum() else "_" for ch in value)


def _vehicle_dimensions(dimensions: dict[str, Any] | None) -> dict[str, float]:
    dimensions = dimensions or {}
    return {
        "length": float(dimensions.get("length", 4.5)),
        "width": float(dimensions.get("width", 2.0)),
        "height": float(dimensions.get("height", 1.6)),
    }


def _format_number(value: float) -> str:
    return f"{value:.6f}".rstrip("0").rstrip(".") or "0"


def _indent(elem, level: int = 0) -> None:
    indent_text = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent_text + "  "
        for child in elem:
            _indent(child, level + 1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent_text
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent_text

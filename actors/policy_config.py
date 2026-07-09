from typing import Any, Dict, Mapping

from actors.closure_levels import actor_closure_level_for_policy
from actors.style_profiles import get_actor_style_profile


_TRIGGER_ROLES = {"trigger", "scripted_trigger", "event_actor", "primary_actor"}
_CONTEXT_ROLES = {"context", "background", "ambient", "replay"}


def build_actor_policy_config(actor: Mapping[str, Any], style: str = "normal") -> Dict[str, Any]:
    role = str(actor.get("role", "context")).lower()
    style_profile = get_actor_style_profile(style)
    policy_mode = _policy_mode_for_role(role)
    closure_level = actor_closure_level_for_policy(policy_mode)

    return {
        "actor_id": _actor_id(actor),
        "role": role,
        "policy_mode": policy_mode,
        "closed_loop_level": closure_level.name,
        "closed_loop": closure_level.to_dict(),
        "conditioning": "reference",
        "style": style_profile.name,
        "style_profile": style_profile.to_dict(),
    }


def _actor_id(actor: Mapping[str, Any]) -> str:
    for key in ("id", "actor_id", "name"):
        value = actor.get(key)
        if value:
            return str(value)
    return "actor"


def _policy_mode_for_role(role: str) -> str:
    if role in _CONTEXT_ROLES:
        return "replay"
    if role in _TRIGGER_ROLES:
        return "scripted_trigger"
    return "reactive_rule_based"

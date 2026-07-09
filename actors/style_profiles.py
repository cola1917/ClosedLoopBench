from dataclasses import asdict, dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class ActorStyleProfile:
    name: str
    desired_time_headway_sec: float
    min_gap_m: float
    reaction_time_sec: float
    yield_ttc_threshold_sec: float
    lane_change_gap_acceptance_m: float
    abort_on_low_ttc: bool

    @classmethod
    def for_style(cls, style: str) -> "ActorStyleProfile":
        return get_actor_style_profile(style)

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


_STYLE_PROFILES: Dict[str, ActorStyleProfile] = {
    "defensive": ActorStyleProfile(
        name="defensive",
        desired_time_headway_sec=2.2,
        min_gap_m=7.5,
        reaction_time_sec=0.7,
        yield_ttc_threshold_sec=4.0,
        lane_change_gap_acceptance_m=12.0,
        abort_on_low_ttc=True,
    ),
    "normal": ActorStyleProfile(
        name="normal",
        desired_time_headway_sec=1.6,
        min_gap_m=5.0,
        reaction_time_sec=1.0,
        yield_ttc_threshold_sec=3.0,
        lane_change_gap_acceptance_m=8.0,
        abort_on_low_ttc=True,
    ),
    "assertive": ActorStyleProfile(
        name="assertive",
        desired_time_headway_sec=1.2,
        min_gap_m=3.5,
        reaction_time_sec=0.8,
        yield_ttc_threshold_sec=2.4,
        lane_change_gap_acceptance_m=6.0,
        abort_on_low_ttc=True,
    ),
    "aggressive": ActorStyleProfile(
        name="aggressive",
        desired_time_headway_sec=0.8,
        min_gap_m=2.0,
        reaction_time_sec=0.5,
        yield_ttc_threshold_sec=1.6,
        lane_change_gap_acceptance_m=4.0,
        abort_on_low_ttc=False,
    ),
    "delayed": ActorStyleProfile(
        name="delayed",
        desired_time_headway_sec=1.6,
        min_gap_m=5.0,
        reaction_time_sec=1.8,
        yield_ttc_threshold_sec=2.2,
        lane_change_gap_acceptance_m=8.0,
        abort_on_low_ttc=True,
    ),
    "noncompliant": ActorStyleProfile(
        name="noncompliant",
        desired_time_headway_sec=0.9,
        min_gap_m=2.5,
        reaction_time_sec=0.6,
        yield_ttc_threshold_sec=1.4,
        lane_change_gap_acceptance_m=3.5,
        abort_on_low_ttc=False,
    ),
}


def available_actor_styles() -> Tuple[str, ...]:
    return tuple(_STYLE_PROFILES.keys())


def get_actor_style_profile(style: str = "normal") -> ActorStyleProfile:
    try:
        return _STYLE_PROFILES[style]
    except KeyError as exc:
        choices = ", ".join(available_actor_styles())
        raise ValueError(f"Unknown actor style '{style}'. Expected one of: {choices}") from exc

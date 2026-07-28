from __future__ import annotations


def test_case_dynamic_actor_ids_prioritize_source_observed_actors():
    from runners.probe_m8_dynamic_batch_lidar import _case_dynamic_actor_ids

    frame = {
        "shared_dynamic_objects": [
            {"actor_id": "background"},
            {"actor_id": "source-a"},
            {"actor_id": "source-b"},
        ]
    }
    cases = _case_dynamic_actor_ids(frame, {"source-a", "source-b"}, [1, 2, "full"])

    assert cases == [
        ("source_prefix_001", ["source-a"]),
        ("source_prefix_002", ["source-a", "source-b"]),
        ("full_003", ["background", "source-a", "source-b"]),
    ]

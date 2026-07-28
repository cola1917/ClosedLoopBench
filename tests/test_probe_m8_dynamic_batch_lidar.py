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
    reversed_cases = _case_dynamic_actor_ids(
        frame,
        {"source-a", "source-b"},
        [1],
        reverse_source_priority=True,
    )
    assert reversed_cases == [("source_prefix_001", ["source-b"])]
    single_cases = _case_dynamic_actor_ids(frame, {"source-a", "source-b"}, ["each"])
    assert single_cases == [
        ("source_single_001_source-a", ["source-a"]),
        ("source_single_002_source-b", ["source-b"]),
    ]

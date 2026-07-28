from __future__ import annotations

import numpy as np


def test_count_uses_oriented_box_coordinates():
    from runners.audit_ncore_dynamic_lidar_points import _count

    points = np.asarray(((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 2.0, 0.0)))
    # A 90-degree yaw maps the first point to local +x, so it is inside the
    # length-2 box; the third remains outside its local half-length.
    assert _count(points, (0.0, 0.0, 0.0), (2.0, 2.0, 2.0), np.pi / 2, np.zeros(3)) == 2


def test_pose_at_chooses_nearest_timestamp():
    from runners.audit_ncore_dynamic_lidar_points import _pose_at

    matrices = np.asarray((np.eye(4), np.eye(4) * 2, np.eye(4) * 3))
    result = _pose_at(matrices, np.asarray((100, 200, 300), dtype=np.uint64), 249)
    assert np.array_equal(result, matrices[1])


def test_interpolate_box_uses_shortest_yaw_arc_without_extrapolation():
    from runners.audit_ncore_dynamic_lidar_points import _interpolate_box_at

    samples = [
        (100, {"centroid": (0.0, 0.0, 0.0), "size": (2.0, 2.0, 2.0), "yaw": np.deg2rad(350.0)}),
        (200, {"centroid": (10.0, 2.0, 4.0), "size": (4.0, 6.0, 8.0), "yaw": np.deg2rad(10.0)}),
    ]
    box, status = _interpolate_box_at(samples, 150)
    assert status == "linearly_interpolated_source_cuboid"
    assert box is not None
    assert np.allclose(box["centroid"], (5.0, 1.0, 2.0))
    assert np.allclose(box["size"], (3.0, 4.0, 5.0))
    assert abs(float(box["yaw"])) < 1e-6
    absent, absent_status = _interpolate_box_at(samples, 99)
    assert absent is None
    assert absent_status == "outside_source_annotation_window"

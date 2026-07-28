from runners.probe_nurec_dynamic_track_points import summarize_track_point_clouds


class _PointCloud:
    def __init__(self, n_points):
        self.n_points = n_points


class _Row:
    def __init__(self, track_id, n_points):
        self.track_id = track_id
        self.point_cloud = _PointCloud(n_points)


def test_summary_retains_zero_point_rows_and_unknown_tracks():
    report = summarize_track_point_clouds(
        [_Row("a", 0), _Row("a", 3), _Row("b", 4), _Row("other", 8)], {"a", "b"}
    )

    assert report["emitted_row_count"] == 3
    assert report["nonempty_row_count"] == 2
    assert report["point_count"] == 7
    assert report["unexpected_track_ids"] == ["other"]
    assert report["tracks"] == [
        {"track_id": "a", "emitted_row_count": 2, "nonempty_row_count": 1, "point_count": 3},
        {"track_id": "b", "emitted_row_count": 1, "nonempty_row_count": 1, "point_count": 4},
    ]

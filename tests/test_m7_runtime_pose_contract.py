import unittest


class M7RuntimePoseContractTests(unittest.TestCase):
    def test_m7_gate_does_not_apply_m6_source_frame_offset(self):
        from runners.run_carla_basic_agent import _m6_runtime_frame_offset_allowed

        self.assertFalse(_m6_runtime_frame_offset_allowed({"m7_actor_pose_audit_required": True}))
        self.assertTrue(_m6_runtime_frame_offset_allowed({}))


if __name__ == "__main__":
    unittest.main()

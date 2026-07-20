import unittest
from unittest import mock


class _Cuda:
    def synchronize(self, _device):
        pass

    def reset_peak_memory_stats(self, _device):
        pass

    def max_memory_allocated(self, _device):
        return 1024


class _Torch:
    cuda = _Cuda()


class _Runtime:
    def __init__(self, _config):
        self.torch = _Torch()
        self.device = "cuda:0"
        self.count = 0

    def predict(self, _observation):
        self.count += 1

    def health_check(self):
        return {
            "real_checkpoint_loaded": True,
            "tensor_warmup_completed": self.count > 0,
            "cuda_device_name": "fake-cuda",
            "torch_version": "test",
            "torch_cuda_version": "test",
        }

    def close(self):
        pass


class TransFuserPPCudaPreflightTests(unittest.TestCase):
    def test_runs_warmup_and_measured_gate_with_bound_identity(self):
        from runners import run_transfuserpp_cuda_preflight as module

        config = {
            "device": "cuda:0",
            "experiment": {"scene_id": "scene", "case_id": "S0", "seed": 41},
            "cuda_gate": {
                "warmup_iterations": 2,
                "measured_iterations": 3,
                "max_peak_memory_bytes": 2048,
                "max_p95_latency_ms": 1000.0,
                "max_p99_latency_ms": 1000.0,
            },
        }
        observation = {
            "frame_id": 10,
            "timestamp": 1.0,
            "synchronization": {"frame_id": 10},
            "run_context": {"run_id": "source"},
        }
        with mock.patch.object(module, "TransFuserPPModelRuntime", _Runtime):
            result = module.run_cuda_preflight(config, observation)
        self.assertEqual(result["status"], "passed")
        self.assertEqual(result["warmup_iterations"], 2)
        self.assertEqual(len(result["latency_ms"]["samples"]), 3)
        self.assertEqual(result["cuda_peak_memory_allocated_bytes"], 1024)
        self.assertEqual(
            result["runtime_identity"]["canonical_sha256"],
            __import__("agents.transfuserpp_contract", fromlist=["cuda_runtime_identity"]).cuda_runtime_identity(config)["canonical_sha256"],
        )


if __name__ == "__main__":
    unittest.main()

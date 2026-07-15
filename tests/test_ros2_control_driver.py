import unittest


class FakeNode:
    def __init__(self):
        self.callback = None
        self.destroyed = False

    def create_subscription(self, message_type, topic, callback, qos):
        self.callback = callback
        self.subscription = (message_type, topic, qos)
        return self.subscription

    def destroy_node(self):
        self.destroyed = True


class FakeVehicleControl:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class FakeCarla:
    VehicleControl = FakeVehicleControl


class Ros2ControlDriverTests(unittest.TestCase):
    def test_safe_stops_until_a_control_command_arrives(self):
        from agents.ros2_control_driver import Ros2ControlDriver

        now = [10.0]
        driver = Ros2ControlDriver(
            node=FakeNode(),
            carla_module=FakeCarla,
            message_type=dict,
            control_topic="/carla/ego/vehicle_control_cmd",
            clock=lambda: now[0],
        )

        control = driver.run_step()

        self.assertEqual(control.throttle, 0.0)
        self.assertEqual(control.brake, 1.0)
        self.assertEqual(driver.diagnostics()["fallback_count"], 1)

    def test_uses_fresh_control_and_falls_back_when_stale(self):
        from agents.ros2_control_driver import Ros2ControlDriver

        now = [10.0]
        node = FakeNode()
        driver = Ros2ControlDriver(
            node=node,
            carla_module=FakeCarla,
            message_type=dict,
            control_topic="/carla/ego/vehicle_control_cmd",
            timeout_sec=0.5,
            clock=lambda: now[0],
        )
        node.callback(
            {
                "throttle": 0.4,
                "steer": -0.1,
                "brake": 0.0,
                "hand_brake": False,
                "reverse": False,
            }
        )

        fresh = driver.run_step()
        now[0] = 10.6
        stale = driver.run_step()

        self.assertEqual(fresh.throttle, 0.4)
        self.assertEqual(fresh.steer, -0.1)
        self.assertEqual(stale.brake, 1.0)
        self.assertEqual(driver.diagnostics()["control_count"], 1)
        self.assertEqual(driver.diagnostics()["fallback_count"], 1)

    def test_invalid_control_is_ignored_and_close_releases_node(self):
        from agents.ros2_control_driver import Ros2ControlDriver

        node = FakeNode()
        driver = Ros2ControlDriver(
            node=node,
            carla_module=FakeCarla,
            message_type=dict,
            control_topic="/control",
        )
        node.callback({"throttle": 2.0, "steer": 0.0, "brake": 0.0})

        self.assertEqual(driver.run_step().brake, 1.0)
        driver.close()
        self.assertTrue(node.destroyed)


if __name__ == "__main__":
    unittest.main()

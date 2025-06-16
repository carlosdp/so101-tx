import base64
import json
import logging
import time
import cv2
import zmq

from lerobot.common.robots.so101_follower import SO101Follower, SO101FollowerConfig

from lerobot.common.robots.network_follower.config_network import NetworkHostConfig, asimov_cameras_config


class NetworkHost:
    def __init__(self, config: NetworkHostConfig):
        self.zmq_context = zmq.Context()
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        self.connection_time_s = config.connection_time_s
        self.watchdog_timeout_ms = config.watchdog_timeout_ms
        self.max_loop_freq_hz = config.max_loop_freq_hz

    def disconnect(self):
        self.zmq_observation_socket.close()
        self.zmq_cmd_socket.close()
        self.zmq_context.term()


def main():
    logging.info("Configuring Network Host")

    robot_config_left = SO101FollowerConfig(
        id="left_arm", port="/dev/ttyACM1", cameras=asimov_cameras_config()
    )
    robot_left = SO101Follower(robot_config_left)
    robot_config = SO101FollowerConfig(id="right_arm", port="/dev/ttyACM0")
    robot = SO101Follower(robot_config)

    logging.info("Connecting Network Host")
    robot_left.connect()
    robot.connect()

    logging.info("Starting HostAgent")
    host_config = NetworkHostConfig()
    host = NetworkHost(host_config)

    last_cmd_time = time.time()
    watchdog_active = False
    logging.info("Waiting for commands...")
    try:
        # Business logic
        start = time.perf_counter()
        duration = 0
        while True:  # while duration < host.connection_time_s:
            loop_start_time = time.time()
            try:
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))
                arm1_data = {key[5:]: value for key, value in data.items() if key.startswith("arm1_")}
                arm2_data = {key[5:]: value for key, value in data.items() if key.startswith("arm2_")}
                _action_sent_left = robot_left.send_action(arm1_data)
                _action_sent = robot.send_action(arm2_data)
                last_cmd_time = time.time()
                watchdog_active = False
            except zmq.Again:
                if not watchdog_active:
                    logging.warning("No command available")
            except Exception as e:
                logging.error("Message fetching failed: %s", e)

            now = time.time()
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Stopping the base."
                )
                watchdog_active = True

            last_observation_left = robot_left.get_observation()
            last_observation_right = robot.get_observation()
            last_observation = {
                **{"arm1_{}".format(key): value for key, value in last_observation_left.items()},
                **{"arm2_{}".format(key): value for key, value in last_observation_right.items()},
            }

            # Encode ndarrays to base64 strings
            for cam_key, _ in robot_left.cameras.items():
                cam_key_spec = "arm1_" + cam_key
                ret, buffer = cv2.imencode(
                    ".jpg", last_observation[cam_key_spec], [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )

                del last_observation[cam_key_spec]

                if ret:
                    last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8")
                else:
                    last_observation[cam_key] = ""

            # Send the observation to the remote agent
            try:
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                logging.info("Dropping observation, no client connected")

            # Ensure a short sleep to avoid overloading the CPU.
            elapsed = time.time() - loop_start_time

            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))
            duration = time.perf_counter() - start
        print("Cycle time reached.")

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Exiting...")
    finally:
        print("Shutting down Network Host.")
        robot.disconnect()
        robot_left.disconnect()
        host.disconnect()

    logging.info("Finished Network Host cleanly")


if __name__ == "__main__":
    main()

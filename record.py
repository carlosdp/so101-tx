# Adapted from LeKiwi example

import time
from threading import Thread, Lock
import sys
import termios
import tty
import select

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from lerobot.common.datasets.utils import hw_to_dataset_features
from so101tx.network_follower.config_network import NetworkClientConfig
from so101tx.network_follower.network_so101_follower import NetworkClient
from lerobot.common.teleoperators.so100_leader import SO100Leader, SO100LeaderConfig

NB_CYCLES_CLIENT_CONNECTION = 250


# --- Key press detection ---
def is_key_pressed():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def get_key():
    if is_key_pressed():
        return sys.stdin.read(1)
    return None


def setup_tty():
    # Save the current terminal settings
    return termios.tcgetattr(sys.stdin)


def restore_tty(old_settings):
    # Restore the original terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# --- Main recording logic ---

HOST = "my-robot.local"  # REPLACE THIS
HF_REPO_PREFIX = "my-username/so101_asimov"  # REPLACE THIS

leader_arm_left_config = SO100LeaderConfig(
    id="leader_left", port="/dev/tty.usbmodem58FA0833831"
)
leader_arm_left = SO100Leader(leader_arm_left_config)
leader_arm_right_config = SO100LeaderConfig(
    id="leader_right", port="/dev/tty.usbmodem58FA0831921"
)
leader_arm_right = SO100Leader(leader_arm_right_config)

robot_config = NetworkClientConfig(remote_ip=HOST, id="network_client")
robot = NetworkClient(robot_config)

action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

dataset = LeRobotDataset.create(
    repo_id=HF_REPO_PREFIX + str(int(time.time())),
    fps=10,
    features=dataset_features,
    robot_type=robot.name,
)

leader_arm_left.connect()
leader_arm_right.connect()
robot.connect()

if (
    not robot.is_connected
    or not leader_arm_left.is_connected
    or not leader_arm_right.is_connected
):
    exit()

print("Starting Network recording")

stop_recording = False
end_episode = False
lock = Lock()


def recording_thread_func():
    global stop_recording, end_episode
    num_episodes = 0
    while True:
        with lock:
            if stop_recording:
                break
        num_episodes += 1
        print("waiting 5 seconds")
        time.sleep(5)
        print(f"Recording episode {num_episodes}...")
        with lock:
            end_episode = False
        while True:
            with lock:
                if end_episode or stop_recording:
                    break
            arm_action_left = leader_arm_left.get_action()
            arm_action_right = leader_arm_right.get_action()
            arm_action_left = {f"arm1_{k}": v for k, v in arm_action_left.items()}
            arm_action_right = {f"arm2_{k}": v for k, v in arm_action_right.items()}

            action = {**arm_action_left, **arm_action_right}
            action_sent = robot.send_action(action)
            observation = robot.get_observation()

            frame = {**action_sent, **observation}
            task = "Dummy Example Task Dataset"

            dataset.add_frame(frame, task)

        print("saving episode")
        dataset.save_episode()
        print(f"Episode {num_episodes} saved.")


print("Starting Network recording.")
print("  - Press 'n' to save the current episode and start a new one.")
print("  - Press 'm' to save the current episode and quit.")

old_settings = setup_tty()
tty.setcbreak(sys.stdin.fileno())

rec_thread = Thread(target=recording_thread_func, daemon=True)
rec_thread.start()

try:
    while rec_thread.is_alive():
        key = get_key()
        if key:
            with lock:
                if key == "n":
                    print("'n' pressed: Finishing current episode.")
                    end_episode = True
                elif key == "m":
                    print("'m' pressed: Finishing current episode and stopping.")
                    stop_recording = True
                    end_episode = True  # Ensure the inner loop breaks
        time.sleep(0.1)  # Small sleep to prevent busy-waiting
finally:
    # This block will run when the try block is exited, including on Ctrl+C
    with lock:
        stop_recording = True  # signal thread to stop
    rec_thread.join(timeout=2)  # wait for thread to finish
    restore_tty(old_settings)

print("Uploading dataset to the hub")
dataset.push_to_hub()

print("\nDisconnecting Teleop Devices and Asimov Client")
robot.disconnect()
leader_arm_left.disconnect()
leader_arm_right.disconnect()

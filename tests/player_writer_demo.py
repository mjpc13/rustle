import time
import shutil
import logging
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_typestore

from utils import DockerWrapper
from components import PlayerConfig, PlayerContainer, WriterConfig, WriterContainer

logging.basicConfig(
        level=logging.INFO,
    )

def main():
    # =========================================================================
    # 0. SETUP DIRS & CHANNELS
    # =========================================================================
    print("=== Setting up Directories & Networks ===")
    
    # Create a clean workspace folder on your host machine for this demo run
    demo_workspace = Path("./workspace/player_writer_demo").resolve()
    if demo_workspace.exists():
        shutil.rmtree(demo_workspace)  # Start fresh
    demo_workspace.mkdir(parents=True, exist_ok=True)
    
    input_bag_dir = demo_workspace / "input_bag"
    outpup_bag_name = "output_bag"
    output_bag_dir = demo_workspace / outpup_bag_name

    # Instantiate our DockerWrapper and create an isolated bridge network
    wrapper = DockerWrapper()
    network_name = "demo_bridge_network"
    wrapper.create_shared_network(network_name)

    # Standard ROS 2 environment variable mapping to isolate DDS traffic
    shared_env = {
        "ROS_DOMAIN_ID": "42",
        "PYTHONUNBUFFERED": "1"
    }

    # Track containers we spawn so we can guarantee cleanup
    player = None
    writer = None

    try:
        # =========================================================================
        # 1. GENERATE DUMMY INPUT BAG
        # =========================================================================

        print("\n=== Create test rosbag ===")
        # Create a typestore and get the string class.
        typestore = get_typestore(Stores.LATEST)
        String = typestore.types['std_msgs/msg/String']

        # Create writer instance and open for writing.
        with Writer(input_bag_dir, version=8) as rb_writer:
            # Add new connection.
            topic = '/test_topic'
            msgtype = String.__msgtype__
            connection = rb_writer.add_connection(topic, msgtype, typestore=typestore)

            # Serialize and write message.
            for i in range(10):
                timestamp = i * 50_000_000
                message = String(f"hello world #{i}")
                rb_writer.write(connection, timestamp, typestore.serialize_cdr(message, msgtype))


        # =========================================================================
        # 2. START THE RECORDER (WRITER)
        # =========================================================================
        print("\n=== Launching the Writer Container ===")

        # Configure the writer to record '/test_topic' to our output folder
        writer_config = WriterConfig(
            output_dir=demo_workspace,
            bag_name=outpup_bag_name,
            topics=["/test_topic"]
        )
        writer = WriterContainer(writer_config, wrapper, network_name, shared_env)

        # Start recording in the background
        writer_id = writer.start()
        print(f"Writer container active (ID: {writer_id[:12]})")

        # Give the recorder 2 seconds to initialize its internal bag writers
        time.sleep(2.0)

        # =========================================================================
        # 3. START THE PLAYER
        # =========================================================================
        print("\n=== Launching the Player Container ===")

        # Configure the player to play our dummy input bag
        player_config = PlayerConfig(
            bag_path=input_bag_dir,
            play_rate=1.0
        )
        player = PlayerContainer(player_config, wrapper, network_name, shared_env)

        # Start playback in the background
        player_id = player.start()
        print(f"Player container active (ID: {player_id[:12]})")

        # =========================================================================
        # 4. WAIT FOR PLAYBACK TO COMPLETE
        # =========================================================================
        print("\n=== Waiting for Playback to Finish ===")

        # Block until the bag finishes playing (exit code 0)
        exit_code = wrapper.wait_for_container(player_id)
        print(f"Player finished playing with exit status: {exit_code}")

    except Exception as e:
        print(f"\nAn error occurred during execution: {e}")

    finally:
        # =========================================================================
        # 5. TEARDOWN
        # =========================================================================
        print("\n=== Tearing down Docker resources ===")

        if player is not None:
            player.stop()
        if writer is not None:
            writer.stop()

        print("Removing the virtual network bridge...")
        wrapper.remove_network(network_name)

    # =========================================================================
    # 6. VALIDATE THE OUTPUT
    # =========================================================================
    print("\n=== Checking the wrote rosbag is correct ===")

    # Create a typestore and get the string class.
    typestore = get_typestore(Stores.LATEST)

    # Create reader instance and open for reading.
    with Reader(output_bag_dir) as reader:
        # Topic and msgtype information is available on .connections list.
        assert len(reader.connections) == 1
        assert reader.connections[0].topic == "/test_topic"
        assert reader.connections[0].msgtype == typestore.types['std_msgs/msg/String'].__msgtype__

        for connection in reader.connections:
            print(connection.topic, connection.msgtype)

        # Iterate over messages.
        for connection, timestamp, rawdata in reader.messages():
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            print(msg)

    print("\nDONE.")


if __name__ == "__main__":
    main()

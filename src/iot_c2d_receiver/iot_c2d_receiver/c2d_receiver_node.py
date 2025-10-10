import os
import asyncio
import datetime
import threading
from typing import Optional
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

try:
    from azure.iot.device.aio import IoTHubDeviceClient
except Exception as e:  # pragma: no cover
    IoTHubDeviceClient = None  # type: ignore

# optional dotenv support
try:
    from dotenv import load_dotenv
except Exception:
    load_dotenv = None  # type: ignore


class C2DReceiverNode(Node):
    def __init__(self):
        super().__init__('c2d_receiver')
       
        package_share_directory = get_package_share_directory('iot_c2d_receiver')
        default_save_dir = os.path.join(package_share_directory, 'waypoints')
        self.declare_parameter('connection_string', '')
        self.declare_parameter('save_dir', default_save_dir)
        self.declare_parameter('env_file', '')

        self._save_dir_root: str = self.get_parameter('save_dir').get_parameter_value().string_value
        self._connection_string: str = self.get_parameter('connection_string').get_parameter_value().string_value
        env_file_param: str = self.get_parameter('env_file').get_parameter_value().string_value

        # If save_dir was provided as empty string via params/launch, fall back to default
        if not self._save_dir_root:
            self._save_dir_root = default_save_dir
            self.get_logger().info(f"Parameter 'save_dir' empty; using default '{self._save_dir_root}'.")

        # Load .env if requested or present and python-dotenv is available
        if load_dotenv is not None:
            try:
                if env_file_param:
                    if os.path.isfile(env_file_param):
                        load_dotenv(dotenv_path=env_file_param, override=False)
                        self.get_logger().info(f"Loaded environment from '{env_file_param}'.")
                    else:
                        self.get_logger().warn(f"env_file '{env_file_param}' not found; skipping .env load.")
                else:
                    # attempt to load .env from current working directory if present
                    cwd_env = os.path.join(os.getcwd(), '.env')
                    if os.path.isfile(cwd_env):
                        load_dotenv(dotenv_path=cwd_env, override=False)
                        self.get_logger().info(f"Loaded environment from '{cwd_env}'.")
            except Exception as e:
                self.get_logger().warn(f"Failed to load .env file: {e}")
        else:
            if env_file_param:
                self.get_logger().warn("Parameter 'env_file' set but python-dotenv is not installed. Install with: pip install python-dotenv")

        # No environment variable fallback; provide via parameter or env_file
        if False:
            env_conn = os.environ.get('IOTHUB_DEVICE_CONNECTION_STRING', '')
            if env_conn:
                self._connection_string = env_conn

        self._client: Optional[IoTHubDeviceClient] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        if IoTHubDeviceClient is None:
            self.get_logger().error('azure-iot-device is not available. Install with: pip install azure-iot-device')

        # kick off the background thread
        self._thread = threading.Thread(target=self._run_thread, daemon=True)
        self._thread.start()
        self.get_logger().info('C2DReceiverNode started. Waiting for C2D messages...')

    def _run_thread(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._main_async())

    async def _main_async(self):
        if IoTHubDeviceClient is None:
            return

        while not self._stop_event.is_set():
            conn = self._connection_string
            if not conn:
                # Check params/env again in case user sets them while running
                conn = self.get_parameter('connection_string').get_parameter_value().string_value
                if not conn:
                    self.get_logger().warn("No connection_string set. Set parameter 'connection_string' or load via 'env_file'. Retrying in 5s...")
                    await asyncio.sleep(5.0)
                    continue
                else:
                    self._connection_string = conn

            try:
                self._client = IoTHubDeviceClient.create_from_connection_string(conn)

                async def message_handler(message):
                    yaml_content = message.data.decode()
                    now = datetime.datetime.now()
                    date_str = now.strftime('%Y-%m-%d')
                    time_str = now.strftime('%H%M%S')
                    save_dir = os.path.join(self._save_dir_root, date_str)
                    os.makedirs(save_dir, exist_ok=True)

                    filename = f'waypoints_{time_str}.yaml'
                    filepath = os.path.join(save_dir, filename)

                    with open(filepath, 'w') as f:
                        f.write(yaml_content)

                    # Ensure root directory exists for current.yaml as well
                    os.makedirs(self._save_dir_root, exist_ok=True)
                    current_path = os.path.join(self._save_dir_root, 'current.yaml')
                    with open(current_path, 'w') as f:
                        f.write(yaml_content)

                    self.get_logger().info(f"Received C2D message id={message.message_id}. Saved '{filepath}' and updated '{current_path}'.")

                self._client.on_message_received = message_handler

                await self._client.connect()
                self.get_logger().info('Connected to IoT Hub. Listening for C2D messages...')

                # Main loop while connected
                while not self._stop_event.is_set():
                    await asyncio.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f'Azure IoT client error: {e}. Reconnecting in 5s...')
                await asyncio.sleep(5.0)
            finally:
                if self._client is not None:
                    try:
                        await self._client.disconnect()
                    except Exception:
                        pass
                    self._client = None

        self.get_logger().info('C2DReceiverNode async loop exiting.')

    def destroy_node(self):
        # signal thread to stop
        self._stop_event.set()
        # allow async loop to stop
        if self._loop is not None:
            def stopper():
                for task in asyncio.all_tasks(loop=self._loop):
                    task.cancel()
            try:
                self._loop.call_soon_threadsafe(stopper)
            except Exception:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=5.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = C2DReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

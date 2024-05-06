import threading
import time
from copy import deepcopy
from typing import Optional, Tuple

import numpy as np
import zmq

from bunny_teleop.init_config import InitializationConfig


class TeleopServer:
    def __init__(self, port: int, host="localhost"):
        # Create socket
        self.ctx = zmq.Context()
        if host == "localhost":
            pub_bind_to = f"tcp://*:{port}"
        else:
            pub_bind_to = f"tcp://{host}:{port}"
        self.pub_socket: Optional[zmq.Socket] = None
        self.init_bind_to = ":".join(
            pub_bind_to.split(":")[:-1] + [str(int(pub_bind_to.split(":")[-1]) + 1)]
        )
        self.pub_bind_to = pub_bind_to

        print(f"Teleop ZMQ Server: Waiting for connection to {pub_bind_to}")

        # Monitor re-initialization with a different thread
        self._lock = threading.Lock()
        self._shared_initialized = False
        self._shared_last_init_config: Optional[InitializationConfig] = None
        self._thread = threading.Thread(target=self.handle_init_config_request)
        self._thread.start()

    def send_teleop_cmd(
            self,
            target_qpos: Tuple[np.ndarray, np.ndarray],
            ee_pose: Tuple[np.ndarray, np.ndarray],
    ):
        if self.pub_socket is not None:
            self.pub_socket.send_pyobj(dict(target_qpos=target_qpos, ee_pose=ee_pose))

    def handle_init_config_request(self):
        try:
            while True:
                init_socket = self.ctx.socket(zmq.REP)
                init_socket.bind(self.init_bind_to)
                print(
                    "Teleop Server: Starting new handling cycle for initialization config."
                )

                init_config_dict = init_socket.recv_json()
                init_socket.close()
                if self.pub_socket is not None:
                    self.pub_socket.close()
                    self.pub_socket = None

                try:
                    init_config = InitializationConfig.from_dict(init_config_dict)
                except TypeError as e:
                    raise ValueError(
                        f"Teleop ZMQ Server: Invalid initialization config. "
                        f"It must be an instance of InitializationConfig.\n"
                        f"{e}"
                    )
                print(f"Teleop Server: receive initialization config")

                with self._lock:
                    self._shared_last_init_config = deepcopy(init_config)
                    self._shared_initialized = False

                while not self._shared_initialized:
                    time.sleep(0.1)

        except KeyboardInterrupt:
            return

    def wait_for_init_config(self):
        print(
            "Teleop ZMQ Server: Waiting for initialization config from teleop client."
        )
        while True:
            with self._lock:
                if self._shared_last_init_config is not None:
                    config = deepcopy(self._shared_last_init_config)
                    return config
            time.sleep(0.1)

    @property
    def initialized(self):
        with self._lock:
            return self._shared_initialized

    @property
    def last_init_config(self):
        with self._lock:
            return self._shared_last_init_config

    def set_initialized(self):
        self.pub_socket = self.ctx.socket(zmq.PUB)
        self.pub_socket.bind(self.pub_bind_to)
        with self._lock:
            self._shared_initialized = True

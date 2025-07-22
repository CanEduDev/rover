import json
import time
from pathlib import Path

import can
from rover import Envelope, bootloader, rover
from rover.can_interface import create_bus


class Flasher:
    def __init__(self, interface, channel, bitrate=125000, config=None):
        self._config = config
        self.online_node_ids = set()
        self.default_timeout_s = 0.1

        # Use intermediate bus variable to avoid linting issues for self.bus.
        self._bus = None
        try:
            self._bus = create_bus(interface, channel, bitrate)
        except Exception as e:
            raise ValueError(f"cannot connect to CAN interface: {e}") from e

        self.bus = self._bus

    def __del__(self):
        if self._bus is not None:
            self._bus.shutdown()

    @property
    def config(self):
        return self._config

    @config.setter
    def config(self, config):
        self._config = config

    def detect_online_nodes(self, restore_comm=True):
        print("Detecting online nodes...")
        self.bus.send(rover.set_action_mode(mode=rover.ActionMode.FREEZE))
        self.bus.send(rover.give_base_number(response_page=1))

        # Check responses
        while True:
            msg = self.bus.recv(timeout=self.default_timeout_s)
            if not msg:
                break
            if msg.is_error_frame:
                continue

            if msg.arbitration_id > rover.BASE_NUMBER:
                self.online_node_ids.add(msg.arbitration_id - rover.BASE_NUMBER)

        if restore_comm:
            self.bus.send(
                rover.set_action_mode(mode=rover.ActionMode.RUN),
                timeout=self.default_timeout_s,
            )

        return self.online_node_ids

    def run(self):
        if self._config is None:
            raise ValueError("run method requires config parameter")

        self.detect_online_nodes()
        if self.online_node_ids == set():
            raise RuntimeError("no nodes found. Please check your CAN connection.")

        self.__check_bus_health()

        # Flash all online nodes
        for node in self._config.nodes:
            id = self._config.get_id(node)

            binary = None
            if not self._config.skip_binaries:
                binary = self._config.get_binary(node)

            config = None
            if not self._config.skip_config:
                config = self._config.get_config(node)

            if id in self.online_node_ids:
                self.__flash_node(id, node=node, binary=binary, config=config)

        print("Flashing successful. Restarting all nodes...")
        self.__restart_all()

    def run_single(self, id, binary_file=None, config_file=None):
        self.detect_online_nodes()

        if id not in self.online_node_ids:
            raise ValueError(
                f"node {id}: node is offline. Found: {self.online_node_ids}"
            )

        binary = None
        config = None

        if binary_file:
            binary = _read_binary(binary_file)

        if config_file:
            config = _read_json(config_file)

        self.__check_bus_health()

        self.__flash_node(id, binary=binary, config=config)
        self.__restart_all()

    def format_fs(self, id):
        prefix = f"node {id}"
        self.detect_online_nodes()

        if id not in self.online_node_ids:
            raise ValueError(
                f"{prefix}: node is offline. Found: {self.online_node_ids}"
            )

        self.__check_bus_health()

        print(f"{prefix}: Starting FS format procedure.")
        self.__single_comm_mode(id)

        print(f"{prefix}: Entering bootloader...")
        self.__enter_bootloader(id)

        print(f"{prefix}: formatting FS...")
        self.__format_fs()

        print(f"{prefix}: Exiting bootloader...")
        self.__exit_bootloader(id)
        self.__restart_all()

    def enter_recovery_mode(self, binary_file, config_file):
        printed_message = False

        # Wait for response, ignore error frames.
        try:
            self.bus.send(rover.default_letter())
            self.bus.send(rover.give_base_number(response_page=1))
        except can.CanError:
            if not printed_message:
                print("If stuck, please power cycle target board.")
                printed_message = True
            pass

        try:
            while True:
                msg = self.bus.recv()
                if not msg.is_error_frame and msg.arbitration_id > rover.BASE_NUMBER:
                    break

        except KeyboardInterrupt:
            raise

        self.__enter_bootloader(0)
        self.detect_online_nodes(restore_comm=False)
        id = self.online_node_ids.pop()
        binary = _read_binary(binary_file)
        config = _read_json(config_file)
        self.__flash_node(id, binary=binary, config=config)

    def __check_bus_health(self):
        print("Checking bus health...")
        start_time = time.monotonic()
        while time.monotonic() - start_time <= 5:
            try:
                self.bus.send(rover.default_letter(), timeout=self.default_timeout_s)
            except can.CanOperationError:
                time.sleep(0.01)

            while (msg := self.bus.recv(timeout=0)) is not None:
                if msg.is_error_frame:
                    raise RuntimeError(
                        "CAN bus is unhealthy. Make sure the bus is terminated with 120 ohm on each end. Flashing on an unhealthy bus may brick your devices."
                    )
                msg = self.bus.recv(timeout=0)

        time.sleep(0.1)  # Give time for TX buffer to clear

        # Flush rx buffer and check for error frames
        while (msg := self.bus.recv(timeout=0)) is not None:
            if msg.is_error_frame:
                raise RuntimeError(
                    "CAN bus is unhealthy. Make sure the bus is terminated with 120 ohm on each end. Flashing on an unhealthy bus may brick your devices."
                )
            msg = self.bus.recv(timeout=0)

    def __flash_node(self, id, node=None, binary=None, config=None):
        if not binary and not config:
            raise ValueError("at least one of binary and config params must be set")

        prefix = f"node {id}"
        if node:
            prefix = f"{node} ({id})"

        print(f"{prefix}: Starting flash procedure.")
        self.__single_comm_mode(id)

        print(f"{prefix}: Entering bootloader...")
        self.__enter_bootloader(id)

        if binary:
            # Erase as many pages as required to fit the application
            print(f"{prefix}: Erasing target flash...")
            self.__flash_erase(len(binary))

            print(f"{prefix}: Flashing binary...")
            self.__flash_program(binary)

        if config:
            print(f"{prefix}: Writing config...")
            self.__write_config(config)

        print(f"{prefix}: Exiting bootloader...")
        self.__exit_bootloader(id)

    def __block_transfer(self, envelope, binary):
        # Init block transfer
        data = [1] + list(len(binary).to_bytes(4, "little")) + [0, 0, 0]
        self.bus.send(
            can.Message(arbitration_id=envelope, dlc=8, data=data, is_extended_id=False)
        )

        try:
            msg = self.bus.recv(timeout=1.0)
            if msg.arbitration_id != envelope:
                raise RuntimeError("wrong response during block transfer init")

        except can.CanError as e:
            raise RuntimeError(f"no bundle request response (1): {e}") from e

        if msg.data[0] != 2:
            raise RuntimeError("wrong response during block transfer init")

        if int.from_bytes(msg.data[1:3], byteorder="little") != 0xFFFF:
            raise RuntimeError("got wrong bundle request, wanted 0xFFFF")

        # Chunk and send data
        chunk_size = 7  # Payload bytes per frame

        chunks = [
            list(binary[i : i + chunk_size]) for i in range(0, len(binary), chunk_size)
        ]

        # Pad last chunk if needed
        last_chunk_length = len(chunks[-1])
        chunks[-1] += [0] * (chunk_size - last_chunk_length)

        current_page_number = 3

        block_transfer_listener = BlockTransferListener(envelope)
        notifier = can.Notifier(self.bus, [block_transfer_listener])

        for chunk in chunks:
            msg = can.Message(
                arbitration_id=envelope,
                dlc=8,
                data=[current_page_number] + chunk,
                is_extended_id=False,
            )

            while not block_transfer_listener.should_abort:
                try:
                    self.bus.send(msg)
                    break
                except can.CanOperationError:
                    continue

            if current_page_number == 3:
                current_page_number = 4
            else:
                current_page_number = 3

        if block_transfer_listener.should_abort:
            raise RuntimeError(
                f"block transfer failed: {block_transfer_listener.abort_reason}"
            )

        t = time.monotonic()
        while not block_transfer_listener.finished_ok and time.monotonic() - t < 10.0:
            pass

        notifier.stop()

        if not block_transfer_listener.finished_ok:
            raise RuntimeError("block transfer timed out")

    def __enter_bootloader(self, id):
        try:
            # Restart target
            self.bus.send(
                rover.set_action_mode(city=id, mode=rover.ActionMode.RESET),
                timeout=self.default_timeout_s,
            )

            time.sleep(0.05)  # Give time for restart

            # Flush rx buffer
            while self.bus.recv(timeout=0) is not None:
                pass

            if id != rover.City.ALL_CITIES:
                # Send default letter, give base number and check for response
                # This way we verify the target is online and ready to receive commands
                restart_timeout = 0.1
                t = time.monotonic()
                while time.monotonic() - t < restart_timeout:
                    try:
                        self.bus.send(rover.default_letter(), timeout=0.01)
                        self.bus.send(rover.give_base_number(city=id), timeout=0.01)
                        msg = self.bus.recv(timeout=0.01)
                    except can.CanError:
                        continue

                    if msg is None:
                        continue

                    if msg.is_error_frame:
                        continue

                    if msg.arbitration_id != rover.BASE_NUMBER + id:
                        raise RuntimeError(
                            f"wrong response during bootloader enter, got {msg.arbitration_id}"
                        )

            self.__assign_bootloader_envelopes(id)

            # Allow target to communicate
            self.bus.send(
                rover.set_comm_mode(city=id, mode=rover.CommMode.COMMUNICATE),
                timeout=self.default_timeout_s,
            )

            msg = can.Message(
                arbitration_id=Envelope.BOOTLOADER_ENTER,
                dlc=0,
                data=[],
                is_extended_id=False,
            )

            self.__send_bootloader_command(msg)
        except can.CanError as e:
            raise RuntimeError(f"entering bootloader failed: {e}") from e

    def __exit_bootloader(self, id):
        msg = can.Message(
            arbitration_id=Envelope.BOOTLOADER_EXIT,
            dlc=0,
            data=[],
            is_extended_id=False,
        )
        try:
            self.__send_bootloader_command(msg)
        except can.CanError as e:
            raise RuntimeError(f"exiting bootloader failed: {e}") from e

        # The power board powers the other boards, so need to sleep longer to compensate for that case.
        if id == rover.City.BATTERY_MONITOR:
            time.sleep(2)
        else:
            time.sleep(0.5)

    def __flash_erase(self, bytes_to_erase):
        msg = can.Message(
            arbitration_id=Envelope.BOOTLOADER_FLASH_ERASE,
            dlc=4,
            data=list(bytes_to_erase.to_bytes(4, "little")),
            is_extended_id=False,
        )
        try:
            self.__send_bootloader_command(msg, timeout=30.0)
        except can.CanError as e:
            raise RuntimeError(f"erasing flash failed: {e}") from e

    def __flash_program(self, binary_data):
        try:
            self.__block_transfer(Envelope.BOOTLOADER_FLASH_PROGRAM, binary_data)
        except RuntimeError as e:
            raise RuntimeError(f"flashing binary failed: {e}") from e
        except can.CanError as e:
            raise RuntimeError(f"flashing binary failed: {e}") from e

    def __format_fs(self):
        msg = can.Message(
            arbitration_id=Envelope.BOOTLOADER_FORMAT_FS,
            dlc=0,
            data=[],
            is_extended_id=False,
        )
        try:
            self.__send_bootloader_command(msg, timeout=10.0)
        except can.CanError as e:
            raise RuntimeError(f"formatting FS failed: {e}") from e

    def __write_config(self, config):
        try:
            binary = json.dumps(config, separators=(",", ":")).encode(encoding="ascii")
            self.__block_transfer(Envelope.BOOTLOADER_FLASH_CONFIG, binary)
        except RuntimeError as e:
            raise RuntimeError(f"writing config failed: {e}") from e
        except can.CanError as e:
            raise RuntimeError(f"writing config failed: {e}") from e

    def __single_comm_mode(self, id):
        self.bus.send(
            rover.set_action_mode(
                city=rover.City.ALL_CITIES, mode=rover.ActionMode.FREEZE
            ),
            timeout=self.default_timeout_s,
        )

        self.bus.send(
            rover.set_comm_mode(city=rover.City.ALL_CITIES, mode=rover.CommMode.SILENT),
            timeout=self.default_timeout_s,
        )

        self.bus.send(
            rover.set_comm_mode(city=id, mode=rover.CommMode.COMMUNICATE),
            timeout=self.default_timeout_s,
        )

    def __restart_all(self):
        # Critical wait to avoid spamming error frames which prevents boards from restarting
        time.sleep(0.05)
        self.bus.send(
            rover.set_action_mode(mode=rover.ActionMode.RESET),
            timeout=self.default_timeout_s,
        )
        time.sleep(0.05)  # Wait for restart

    def __assign_bootloader_envelopes(self, node_id):
        bootloader_assignments = bootloader.generate_assignments(node_id)
        for assignment in bootloader_assignments:
            self.bus.send(
                rover.assign_envelope(node_id, assignment.envelope, assignment.folder),
                timeout=self.default_timeout_s,
            )

    def __send_bootloader_command(self, msg, timeout=None):
        if not timeout:
            timeout = self.default_timeout_s

        self.bus.send(msg)

        t = time.monotonic()

        response = None

        while time.monotonic() - t < timeout:
            response = self.bus.recv(timeout=timeout)

            if response and response.arbitration_id == Envelope.BOOTLOADER_COMMAND_ACK:
                break

        if response is None:
            raise RuntimeError(f"no ACK received after {timeout} seconds")

        if response.arbitration_id != Envelope.BOOTLOADER_COMMAND_ACK:
            raise RuntimeError(f"wrong ACK received, got frame: {response}")

        command_id = int.from_bytes(response.data[1:5], byteorder="little")

        if response.data[0] == 1:  # NACK value
            raise RuntimeError(f"NACK received for command {hex(command_id)}")

        if command_id != msg.arbitration_id:
            raise RuntimeError(
                f"wrong ACK for command, expected: {hex(msg.arbitration_id)}, got {hex(command_id)}"
            )


class FlasherConfig:
    def __init__(self, flasher_conf, binary_dir, skip_binaries=False, skip_config=True):
        try:
            f = _read_binary(flasher_conf)
            self.json = json.loads(f)
        except Exception:
            raise

        for name, val in self.json.items():
            if "id" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "id" key'
                )

            if "binary" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "binary" key'
                )

            if "config" not in val:
                raise ValueError(
                    f'invalid flasher configuration, {name} is missing "config" key'
                )

        self.skip_binaries = skip_binaries
        self.skip_config = skip_config

        self.node_binary_map = {}

        if self.skip_binaries:
            return

        try:
            for node in self.json:
                file_path = Path(binary_dir, self.json[node]["binary"])
                self.node_binary_map[node] = _read_binary(file_path)

        except Exception:
            raise

    @property
    def nodes(self):
        return self.json.keys()

    def get_id(self, node):
        return self.json[node]["id"]

    def get_config(self, node):
        return self.json[node]["config"]

    def get_binary(self, node):
        return self.node_binary_map[node]

    def __repr__(self):
        return str(self.json)


def _read_binary(file):
    try:
        with Path(file).open("rb") as f:
            return f.read()
    except Exception as e:
        raise ValueError(f"couldn't read {file}: {e}") from e


def _read_json(file):
    try:
        with Path(file).open() as f:
            return json.load(f)
    except Exception as e:
        raise ValueError(f"couldn't read {file}: {e}") from e


class BlockTransferListener(can.Listener):
    def __init__(self, envelope):
        self.envelope = envelope
        self.should_abort = False
        self.abort_reason = None
        self.finished_ok = False

    def on_message_received(self, msg):
        if msg.arbitration_id == self.envelope:
            if msg.data[0] == 5:
                self.should_abort = True
                self.abort_reason = "received abort page"
                return

            if (
                msg.data[0] == 2
                and int.from_bytes(msg.data[1:3], byteorder="little") == 0
            ):
                self.finished_ok = True

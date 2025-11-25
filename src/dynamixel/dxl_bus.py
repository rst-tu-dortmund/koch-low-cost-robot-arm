# Copyright 2025, Institute for Control Theory and Systems Engineering, 
# TU Dortmund University
#
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, 
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” 
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
# POSSIBILITY OF SUCH DAMAGE.

"""
@brief Generic bus wrapper and grouped I/O helpers for Dynamixel devices.

Provides thin convenience layers atop the official `dynamixel_sdk` to manage port/protocol setup via a
context manager and to perform generic read/write of control-table fields.

@see DxlBus for connection management and register access.
@see SyncGroup for grouped synchronous I/O.
"""


import os

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,

    COMM_SUCCESS,
)

from .motor_specs.base import ControlField, DynamixelMotor

from typing import (
    List,
    Dict,
)

class SyncGroup:
    """
    @brief Batch read/write wrapper for a single control-table field across multiple motors.

    Creates paired `GroupSyncWrite` and `GroupSyncRead` handles for the same address and
    data length. Each provided motor ID is registered for both read and write operations.

    @note Uses little-endian, signed representation when writing multi-byte values.
    """

    def __init__(self, port_handler: PortHandler, packet_handler: PacketHandler, 
                 control_field: ControlField, motor_ids: List[int]):
        """
        @brief Initialize a synchronous group for the given control field and motor IDs.

        @param port_handler PortHandler Opened port handler from the SDK.
        @param packet_handler PacketHandler Packet handler matching the protocol version.
        @param control_field ControlField Target control-table field.
        @param motor_ids List[int] Motor IDs to include in the group.
        """
        ## Register start address.
        self.address = control_field.address
        ## Control field data length
        self.data_length = control_field.size
        ## Motor IDs in this group
        self.motor_ids = motor_ids

        ## GroupSyncWrite handle for batched writes.
        self._group_sync_write = GroupSyncWrite(port_handler, packet_handler, self.address, self.data_length)
        ## GroupSyncRead handle for batched reads.
        self._group_sync_read = GroupSyncRead(port_handler, packet_handler, self.address, self.data_length)

        for id in self.motor_ids:
            self._group_sync_read.addParam(id)
            self._group_sync_write.addParam(id, bytes(self.data_length))

    def write(self, data: Dict[int, int]):
        """
        @brief Write per-motor values to the group's control field in one packet.

        @param data Dict[int,int] Mapping {motor ID: value} to write (signed, little-endian).
        """
        for id, value in data.items():
            self._group_sync_write.changeParam(id, value.to_bytes(self.data_length, 'little', signed=True))

        res = self._group_sync_write.txPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError(f"Failed to write data to motors!")
        
    def read(self) -> Dict[int, int]:
        """
        @brief Read the group's control field for all registered motors.

        @return Dict[int,int] Mapping {motor ID: raw integer value read}.
        """
        res = self._group_sync_read.txRxPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError(f"Failed to read data from motors!")
        
        output = {}
        for id in self.motor_ids:
            if self._group_sync_read.isAvailable(id, self.address, self.data_length):
                output[id] = self._group_sync_read.getData(id, self.address, self.data_length)

        return output
    

class DxlBus:
    """
    @brief Lightweight bus wrapper to open/close the port and access Dynamixel registers.

    Provides context-manager semantics, helpers to read/write registers via `ControlField` descriptors,
    and support for constructing and caching `SyncGroup` instances.

    @note Assumption: A single physical port is used; `_get_active_port` searches `/dev` for entries
          containing `ttyUSB` or `ttyACM` when no port name is provided.
    """

    def __init__(self, port_name: str = "", baudrate: int = 57_600, protocol_version: float = 2.0, motors: List[DynamixelMotor] = []):
        """
        @brief Create a bus object and prepare SDK handlers (port unopened).

        @param port_name str OS device path. If empty, try to auto-detect an active port.
        @param baudrate int Port baud rate configured on the motors.
        @param protocol_version float Dynamixel protocol version (e.g., 2.0).
        @param motors List[DynamixelMotor] Optional motor ids bound to this bus.

        @note The port is not opened until `connect()` or entering the context manager.
        @note Assumption: `motors` may be mutated externally; the default `[]` is shared.
        """
        ## Port name (e.g., `/dev/ttyUSB0`).
        self.port_name = port_name if port_name else self._get_active_port()
        ## Port baud rate configured on the motors.
        self.baudrate = baudrate
        ## Port handler for serial communication.
        self.port_handler = PortHandler(self.port_name)
        ## Packet handler for protocol communication.
        self.packet_handler = PacketHandler(protocol_version)
        ## Motors bound to this bus.
        self.motors: List[DynamixelMotor] = motors

        ## Cached sync groups by name.
        self._sync_groups: Dict[str, SyncGroup] = {}
        ## Connection state flag.
        self._is_open = False

    def __del__(self):
        """
        @brief Destructor to ensure the port is closed on object deletion.
        """
        self.disconnect()

    def __enter__(self):
        """
        @brief Context manager entry: open the port.
        """
        self.connect()
        
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """
        @brief Context manager exit: close the port.
        """
        self.disconnect()

        if exc_type is not None:
            raise exc_value

    def connect(self):      
        """
        @brief Open the serial port for communication.
        """  
        if not self.port_handler.openPort():
            raise Exception(f'Failed to open port {self.port_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise Exception(f'failed to set baudrate to {self.baudrate}')
        
        self._is_open = True
        
    def disconnect(self):
        """
        @brief Close the serial port if it is open.
        """
        if self._is_open:
            self.port_handler.closePort()
            self._is_open = False

    def write_reg(self, motor_id, control_field: ControlField, value: int):
        """
        @brief Write a single control-table entry by size-aware API.

        @param motor_id int Target motor ID.
        @param control_field ControlField Target control-table field.
        @param value int Value to write (interpreted unsigned/signed by SDK routine).

        @note Uses 1-, 2-, or 4-byte write based on `control_field.size`.
        """

        if control_field.size == 1:
            res, err = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        elif control_field.size == 2:
            res, err = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        elif control_field.size == 4:
            res, err = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        if res != COMM_SUCCESS or err:
            raise RuntimeError(f"Write fail id={motor_id} res={res} err={err}")

    def read_reg(self, motor_id: int, control_field: ControlField) -> int:
        """
        @brief Read a single control-table entry by size-aware API.

        @param motor_id int Target motor ID.
        @param control_field ControlField Target control-table field.
        @return int Raw integer value read.

        @note Uses 1-, 2-, or 4-byte read based on `control_field.size`.
        """

        if control_field.size == 1:
            val, res, err = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        elif control_field.size == 2:
            val, res, err = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        elif control_field.size == 4:
            val, res, err = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        if res != COMM_SUCCESS or err:
            raise RuntimeError(f"Read fail id={motor_id} res={res} err={err}")
        
        return val

    def make_sync_group(self, sync_group_name: str, control_field: ControlField, ids: List[int]) -> SyncGroup:
        """
        @brief Create (or return cached) `SyncGroup` for a field and motor set.

        @param sync_group_name str Key under which the group is cached.
        @param control_field ControlField Target control-table field.
        @param ids List[int] Motor IDs to include in the group.
        @return SyncGroup The created or cached group instance.
        """

        key = sync_group_name
        if key not in self._sync_groups:
            self._sync_groups[key] = SyncGroup(self.port_handler, self.packet_handler, control_field, ids)
        return self._sync_groups[key]
    
    def get_sync_group(self, sync_group_name: str) -> SyncGroup:
        """
        @brief Retrieve a previously created `SyncGroup` by name.

        @param sync_group_name str Cache key used in `make_sync_group`.
        @return SyncGroup The requested group.
        """

        if sync_group_name not in self._sync_groups:
            raise KeyError(f"Sync group '{sync_group_name}' does not exist.")
        return self._sync_groups[sync_group_name]

    def find_active_motors(self, id_range = range(1, 253)) -> List[int]:
        """
        @brief Ping a range of IDs and return those that respond successfully.

        @param id_range range Iterable of IDs to ping (default 1..252).
        @return List[int] Detected motor IDs.
        """

        found_ids: List[int] = []
        for motor_id in id_range:
            model, res, err = self.packet_handler.ping(self.port_handler, motor_id)
            if res == COMM_SUCCESS and not err:
                found_ids.append(motor_id)
        return found_ids

    def _get_active_port(self) -> str:
        """
        @brief Select a connected serial device under `/dev`.

        Scans for entries containing `ttyUSB` or `ttyACM` and returns the first match.

        @return str Device path such as `/dev/ttyUSB0`.

        @note Assumption: Linux-like environment with `/dev` available.
        """

        for port_name in os.listdir('/dev'):
            if 'ttyUSB' in port_name or 'ttyACM' in port_name:
                full_port_name = '/dev/' + port_name
                return full_port_name
        
        raise RuntimeError("No active port found. Please connect a Dynamixel device.")
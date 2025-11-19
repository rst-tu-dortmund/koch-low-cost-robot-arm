# `dxl_bus.py` — Dynamixel Bus Wrapper

A small, ergonomic wrapper around the official [`dynamixel-sdk`](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) to:

* open/close a serial bus
* read/write single registers via typed control-table fields
* perform efficient group (sync) read/write of the same register across multiple motors

Integrates with your motor spec types (`ControlField`, `DynamixelMotor`) from the [motor_specs](motor_specs.md) module.
Use the [utils.py](utils.md) module for unit handling and data conversion.

---

## Contents

* [Concepts](#concepts)
* [API Reference](#api-reference)

  * [`DxlBus`](#class-dxlbus)
  * [`SyncGroup`](#class-syncgroup)

---

## Concepts

* **ControlField** — lightweight descriptor for a register (address, size, name), defined in [motor_specs](motor_specs.md).
* **DxlBus** — connection wrapper around `PortHandler`/`PacketHandler` that opens/closes the serial port, exposes `read_reg`/`write_reg` using `ControlField`s, and manages `SyncGroup`s.
* **SyncGroup** — wrapper over `GroupSyncWrite` / `GroupSyncRead` for a shared `(address, size)` across multiple IDs (e.g., “Goal Position” for a set of motors).

---

## API Reference

### Class: `DxlBus`

A context-managed bus connection. Prefer:

```python
with DxlBus('/dev/ttyUSB0', 1_000_000) as bus:
    ...
```

#### Constructor

```python
DxlBus(
    port_name: str = "",
    baudrate: int = 57_600,
    protocol_version: float = 2.0,
    motors: list[DynamixelMotor] = []
)
```

* **port_name**: serial device path (e.g., `/dev/ttyUSB0`). If empty, the first matching `ttyUSB*`/`ttyACM*` is auto-selected.
* **baudrate**: bus baudrate (default `57_600`).
* **protocol_version**: Dynamixel protocol (default `2.0`).
* **motors**: optional list your app keeps for convenience; not used internally by the bus.

---

#### `connect`

```python
connect() -> None
```

Open the port and set the baudrate.

* **Errors:** raises `RuntimeError`/`OSError` if the device cannot be opened or configured.

---

#### `disconnect`

```python
disconnect() -> None
```

Close the port if open.

---

#### `write_reg`

```python
write_reg(motor_id: int, field: ControlField, value: int) -> None
```

Write a raw integer `value` to a single motor register.

* **Args:** `motor_id`, `field` (provides address and size), `value` (raw DXL units).
* **Notes:** value range is not validated here; the device/SDK enforces limits.
* **Errors:** raises `RuntimeError` on SDK/device error.

---

#### `read_reg`

```python
read_reg(motor_id: int, field: ControlField) -> int
```

Read a single register from one motor.

* **Args:** `motor_id`, `field`.
* **Returns:** raw integer in DXL units.
* **Errors:** raises `RuntimeError` on SDK/device error.

---

#### `make_sync_group`

```python
make_sync_group(name: str, field: ControlField, ids: list[int]) -> SyncGroup
```

Create (or return a cached) sync group for a shared `(address, size)` across `ids`.

* **Args:** `name` (cache key), `field`, `ids`.
* **Returns:** `SyncGroup` bound to `field` and `ids`.
* **Notes:** creating a group with an existing `name` returns the cached instance.

---

#### `get_sync_group`

```python
get_sync_group(name: str) -> SyncGroup
```

Retrieve a previously created sync group.

* **Errors:** raises `KeyError` if no group is cached under `name`.

---

#### `find_active_motors`

```python
find_active_motors(id_range: range = range(1, 253)) -> list[int]
```

Ping each ID in `id_range` and return those that respond.

* **Returns:** list of responsive motor IDs.
* **Notes:** useful for discovery on a new bus.

---

### Class: `SyncGroup`

A grouped view over a single control field for multiple motor IDs. Uses the same `(address, size)` for all operations.

#### `write`

```python
write(data: dict[int, int]) -> None
```

Write the configured field for selected IDs.

* **Args:** `data` — mapping `{motor_id: raw_value}` in DXL units.
* **Notes:** partial updates allowed; only IDs present in `data` are written.
* **Errors:** raises `RuntimeError` on SDK/device error.

---

#### `read`

```python
read() -> dict[int, int]
```

Read the configured field from all IDs in the group.

* **Returns:** mapping `{motor_id: raw_value}` in DXL units.
* **Errors:** raises `RuntimeError` on SDK/device error.
---
[Back to Overview](README.md)
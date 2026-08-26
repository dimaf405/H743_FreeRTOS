# DroneCAN DSDL provenance

- DSDL upstream: https://github.com/PX4/DSDL
- DSDL commit: `993be80a62ec957c01fb41115b83663959a49f46`
- Generator: https://github.com/DroneCAN/dronecan_dsdlc
- Generator commit: `431170fa4bfe2212b516b8f33bdc796267907f1c`
- License: MIT; see `LICENSE`.

Only the pinned canonical DSDL inputs required by the H743 product are
vendored:

- `uavcan.protocol.NodeStatus`
- `uavcan.protocol.GetNodeInfo` and its version dependencies
- `uavcan.protocol.dynamic_node_id.Allocation`
- `uavcan.equipment.ahrs.MagneticFieldStrength`
- `uavcan.equipment.ahrs.MagneticFieldStrength2`

`Dima/lib/protocols/dronecan/dronecan_contract.json` is the single selected
type/parameter/subscription contract. `tools/dronecan/generate_contract.py`
validates every DSDL input hash, provisions the pinned compiler and Python
dependencies, then generates all C/H codecs and build catalogues into
`build/generated/dronecan`. Generated C/H output is intentionally not stored
in this source directory.

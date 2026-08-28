# DroneCAN DSDL provenance

- DSDL upstream: https://github.com/PX4/DSDL
- DSDL commit: `993be80a62ec957c01fb41115b83663959a49f46`
- Generator: https://github.com/DroneCAN/dronecan_dsdlc
- Generator commit: `431170fa4bfe2212b516b8f33bdc796267907f1c`
- License: MIT; see `LICENSE`.

Only the pinned canonical DSDL source closure required by the H743 product is
vendored under `dsdl/uavcan`. The build discovers every `.uavcan` file in that
tree, so this provenance document does not maintain a second message list.

`tools/dronecan/generate_contract.py` provisions the pinned compiler and Python
dependencies, then generates the C/H codecs, typed runtime contract and Make
source fragment into `build/generated/dronecan`. It does not generate or retain
a DroneCAN JSON contract or catalogue in either the source tree or build tree.
Product parameters are independently defined in
`Dima/middleware/parameters/definitions/module_dronecan.yaml` and use the same
parameter generation chain as every other `module_*.yaml`. Generated C/H output
is intentionally not stored in this source directory.

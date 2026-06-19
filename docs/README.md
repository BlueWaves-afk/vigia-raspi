# VIGIA Documentation Index

Canonical entry point for `vigia-raspi` (edge / blackbox) docs. For the linked
knowledge-graph view, open `.wiki/00_Index.md` in Obsidian. The authoritative
implementation-status tracker is [`.claude/design/GAP_TRACKER.md`](../.claude/design/GAP_TRACKER.md).

## Current architecture

| Doc | Topic |
|-----|-------|
| [system_architecture.md](system_architecture.md) | Overall system architecture |
| [hardware-architecture.md](hardware-architecture.md) | BOM, Pi 5 + Pico + sensors wiring |
| [device_provisioning.md](device_provisioning.md) | Cert / key provisioning |
| [pico-pi-bringup.md](pico-pi-bringup.md) | First-boot bring-up |
| [sensor-fusion-plan.md](sensor-fusion-plan.md) | ISS / RRI fusion design |
| [app_dashcam_integration.md](app_dashcam_integration.md) | Phone ↔ Pi BLE integration |
| [demo-run-guide.md](demo-run-guide.md) | Running the demo |

Design contracts (machine-readable specs): [`.claude/design/`](../.claude/design/)
— ROS 2 node contracts, Pico firmware contracts, ONNX vision engine, DePIN signing,
anti-death, BLE transport.

## Performance / investigations

| Doc | Topic |
|-----|-------|
| [latency-deep-dive.md](latency-deep-dive.md) | End-to-end latency |
| [fps-investigation-2to7.md](fps-investigation-2to7.md) | FPS tuning |
| [openvino-acl-a72-issue.md](openvino-acl-a72-issue.md) | ACL EP notes |
| [power-distribution.md](power-distribution.md) | Power budget |

## Historical (pre-M12)

> [!NOTE]
> These describe the FastAPI + Mosquitto + PostgreSQL backend that **M12 replaced
> with AWS IoT Core + Lambda + DynamoDB**. Kept for provenance. The current backend
> lives in the `vigia-amazon` repo; see `.wiki/09_Cloud_Pipeline.md`.

- [m7-event-logging-database-plan.md](m7-event-logging-database-plan.md)
- [m7-event-logging-implementation.md](m7-event-logging-implementation.md)
- [HANDOFF.md](HANDOFF.md)

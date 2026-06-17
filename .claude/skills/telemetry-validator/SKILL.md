# Skill Playbook: Telemetry Pipeline Validation

## Operational Context
Apply this specialized playbook whenever editing or testing code relative to packet structures, transmission frames, or network payload updates.

## Verification Checklist
1. Verify that all outgoing telemetry packages undergo rigid signing mechanisms before transmission.
2. Confirm that system data serialization operations utilize deterministic layouts optimized for memory-aligned hardware.
3. Validate that non-blocking network streams are used to avoid hanging loops.

## Local Automation Verification Hook
Before declaring a task completed, execute this shell operation to verify safety constraints:
`make validate-buffers`
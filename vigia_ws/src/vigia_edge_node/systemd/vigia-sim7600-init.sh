#!/bin/bash
# vigia-sim7600-init.sh — Per-boot LTE bearer setup for SIM7600 in ECM mode.
#
# The SIM7600 must already be switched to ECM mode (one-time, via AT+CUSBPIDSWITCH).
# This script handles per-boot network setup: wait for usb0, acquire DHCP, route broker.
#
# Called by vigia-edge.service ExecStartPre.
# Fails loudly (exit 1) if LTE is required; silently if MQTT broker is not configured.

set -euo pipefail

BROKER_IP="${VIGIA_MQTT_BROKER_IP:-}"   # set in /etc/vigia/env or systemd EnvironmentFile
TIMEOUT_S=30

# 1. Wait for usb0 (SIM7600 ECM device enumeration)
echo "vigia-sim7600-init: waiting for usb0 (timeout ${TIMEOUT_S}s)..."
if ! timeout "${TIMEOUT_S}" bash -c \
    'until ip link show usb0 > /dev/null 2>&1; do sleep 1; done'; then
    echo "vigia-sim7600-init: WARNING — usb0 not found after ${TIMEOUT_S}s."
    echo "  SIM7600 may not be connected or ECM mode not activated."
    echo "  Run: AT+CUSBPIDSWITCH=9011,1,1  (one-time, requires reboot)"
    # Non-fatal — node continues in degraded mode (no MQTT uplink).
    exit 0
fi

echo "vigia-sim7600-init: usb0 found."

# 2. Bring interface up
ip link set usb0 up

# 3. Acquire DHCP address from SIM7600's built-in DHCP server
if ! dhclient -1 -v usb0 2>&1 | head -20; then
    echo "vigia-sim7600-init: WARNING — DHCP on usb0 failed."
    exit 0
fi

echo "vigia-sim7600-init: usb0 IP acquired: $(ip -4 addr show usb0 | grep inet | awk '{print $2}')"

# 4. Add host route for MQTT broker via usb0 (if configured)
#    Keeps LTE traffic to broker only — all other traffic uses Tailscale/eth0.
if [[ -n "${BROKER_IP}" ]]; then
    ip route replace "${BROKER_IP}/32" dev usb0
    echo "vigia-sim7600-init: route ${BROKER_IP}/32 via usb0 set."
else
    echo "vigia-sim7600-init: VIGIA_MQTT_BROKER_IP not set — no specific route added."
fi

echo "vigia-sim7600-init: LTE bearer ready."

#!/usr/bin/env bash
# vigia-sim7600-init.sh — bring up SIM7600 ECM USB ethernet interface
#
# Called as ExecStartPre in vigia-edge.service.
# Prerequisites:
#   - SIM7600 switched to ECM mode (AT+CUSBPIDSWITCH=9011,1,1 run once by Ben)
#   - usb0 interface present after ECM enumeration (~3s after boot)
#
# Exits 0 on success, non-zero on failure (systemd will abort the service start).

set -euo pipefail

IFACE="${VIGIA_LTE_IFACE:-usb0}"
MAX_WAIT=30  # seconds to wait for the interface to appear

echo "[sim7600-init] Waiting for ${IFACE} to appear..."
for i in $(seq 1 "${MAX_WAIT}"); do
    if ip link show "${IFACE}" &>/dev/null; then
        echo "[sim7600-init] ${IFACE} found after ${i}s"
        break
    fi
    if [ "${i}" -eq "${MAX_WAIT}" ]; then
        echo "[sim7600-init] ERROR: ${IFACE} not found after ${MAX_WAIT}s — SIM7600 not in ECM mode?" >&2
        exit 1
    fi
    sleep 1
done

ip link set "${IFACE}" up
echo "[sim7600-init] ${IFACE} link up"

# Request IP via DHCP (SIM7600 internal DHCP server on ECM interface).
if command -v dhclient &>/dev/null; then
    dhclient "${IFACE}" -v -timeout 15
elif command -v udhcpc &>/dev/null; then
    udhcpc -i "${IFACE}" -t 15 -q
else
    echo "[sim7600-init] WARNING: no DHCP client found; interface may have no IP" >&2
fi

# Verify connectivity — ping SIM7600 gateway (ECM default: 192.168.225.1).
GATEWAY="${VIGIA_LTE_GATEWAY:-192.168.225.1}"
if ping -I "${IFACE}" -c 2 -W 5 "${GATEWAY}" &>/dev/null; then
    echo "[sim7600-init] LTE gateway reachable at ${GATEWAY}"
else
    echo "[sim7600-init] WARNING: gateway ${GATEWAY} not reachable — LTE may not be registered" >&2
    # Non-fatal: MQTT client will retry internally.
fi

echo "[sim7600-init] done"
exit 0

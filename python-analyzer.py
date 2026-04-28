"""
Nimbrace swing plane viewer.
Connects to Nimbrace BLE device, stays connected, displays swings live.

Install: pip install bleak matplotlib numpy
"""

import asyncio
import struct
import time
from dataclasses import dataclass

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner

DEVICE_NAME = "Nimbrace"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


@dataclass
class Swing:
    back_angle_deg: float   # tilt from horizontal
    down_angle_deg: float   # tilt from horizontal
    back_dur_ms: int
    down_dur_ms: int
    back_ratios: np.ndarray
    down_ratios: np.ndarray


class PacketParser:
    HEADER_SWING = 0xAA
    HEADER_HEARTBEAT = 0xBB
    SWING_HEADER_LEN = 1 + 4 + 4 + 2 + 2 + 2 + 2

    def __init__(self, on_swing, on_heartbeat):
        self.on_swing = on_swing
        self.on_heartbeat = on_heartbeat
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        while self.buf:
            b = self.buf[0]
            if b == self.HEADER_HEARTBEAT:
                if len(self.buf) < 2:
                    return
                del self.buf[:2]
                self.on_heartbeat()
            elif b == self.HEADER_SWING:
                if len(self.buf) < self.SWING_HEADER_LEN:
                    return
                (_h, a_back, a_down, bms, dms, nb, nd) = struct.unpack_from(
                    "<BffHHHH", self.buf, 0)
                total = self.SWING_HEADER_LEN + nb + nd
                if nb > 1000 or nd > 1000:
                    del self.buf[:1]
                    continue
                if len(self.buf) < total:
                    return
                back_r = np.frombuffer(self.buf[self.SWING_HEADER_LEN:self.SWING_HEADER_LEN + nb],
                                       dtype=np.uint8).astype(np.float32) / 255.0
                down_r = np.frombuffer(self.buf[self.SWING_HEADER_LEN + nb:total],
                                       dtype=np.uint8).astype(np.float32) / 255.0
                del self.buf[:total]
                self.on_swing(Swing(a_back, a_down, bms, dms, back_r, down_r))
            else:
                del self.buf[:1]


class Viewer:
    def __init__(self):
        self.fig, axes = plt.subplots(1, 2, figsize=(12, 6),
                                      gridspec_kw={'width_ratios': [1, 1.3]})
        self.ax_planes, self.ax_ratio = axes
        self.fig.canvas.manager.set_window_title("Nimbrace swing plane")
        self.status_text = self.fig.text(0.5, 0.97, "Scanning for Nimbrace...",
                                         ha='center', fontsize=11, color='#666')
        self.last_hb_ms = 0
        self._draw_empty()

    def set_status(self, msg, color='#666'):
        self.status_text.set_text(msg)
        self.status_text.set_color(color)
        self.fig.canvas.draw_idle()

    def heartbeat(self):
        self.last_hb_ms = int(time.time() * 1000)
        print(f"  [heartbeat at {self.last_hb_ms}ms]")

    def _draw_empty(self):
        self.ax_planes.clear()
        self.ax_planes.set_xlim(-1.3, 1.7)
        self.ax_planes.set_ylim(-0.4, 1.6)
        self.ax_planes.set_aspect('equal')
        self.ax_planes.axis('off')
        self.ax_planes.set_title("Swing planes  (rear view)")

        self.ax_ratio.clear()
        self.ax_ratio.set_xlim(0, 2.1)
        self.ax_ratio.set_ylim(0, 1)
        self.ax_ratio.set_xlabel("Phase progress")
        self.ax_ratio.set_ylabel("Off-plane ratio")
        self.ax_ratio.set_title("On-plane quality")
        self.ax_ratio.axhline(0.25, color='gray', ls='--', lw=0.7)
        self.fig.tight_layout(rect=[0, 0, 1, 0.95])

    def update(self, swing):
        self._draw_planes(swing)
        self._draw_ratios(swing)
        self.fig.canvas.draw_idle()

    def _draw_planes(self, swing):
        ax = self.ax_planes
        ax.clear()
        ax.set_xlim(-1.3, 1.7)
        ax.set_ylim(-0.4, 1.6)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title("Swing planes  (rear view)")

        # Stick figure
        head_y = 1.4
        ax.add_patch(plt.Circle((0, head_y), 0.1, fc='#dddddd', ec='black', lw=1.5))
        ax.plot([0, 0], [0.0, head_y - 0.1], 'k-', lw=2)
        ax.plot([-0.25, 0.25], [head_y - 0.4, head_y - 0.4], 'k-', lw=2)
        ax.plot([-0.18, 0.0, 0.18], [0.0, 0.0, 0.0], 'k-', lw=2)

        # Plane lines from trail-side shoulder.
        # Angle is tilt from HORIZONTAL: 0° = flat (baseball), 90° = vertical (chop).
        sx, sy = 0.25, head_y - 0.4
        plane_len = 1.4

        def plane_line(angle_deg, color, label, valid):
            if not valid:
                return
            theta = np.deg2rad(angle_deg)
            # 0° tilt -> line goes horizontally outward (dx=1, dy=0)
            # 90° tilt -> line goes straight down (dx=0, dy=-1)
            dx = np.cos(theta)
            dy = -np.sin(theta)
            ax.plot([sx, sx + dx * plane_len], [sy, sy + dy * plane_len],
                    color=color, lw=3, label=label)

        plane_line(swing.back_angle_deg, '#1f77b4',
                   f'Backswing: {swing.back_angle_deg:.1f}° from horiz',
                   swing.back_dur_ms > 0)
        plane_line(swing.down_angle_deg, '#d62728',
                   f'Downswing: {swing.down_angle_deg:.1f}° from horiz',
                   swing.down_dur_ms > 0)
        ax.legend(loc='upper left', fontsize=10, frameon=False)

        # Reference labels for context
        ax.text(1.45, 0.0, '0° (flat)', fontsize=8, color='#999', va='center')
        ax.text(0.32, -0.32, '90° (vertical)', fontsize=8, color='#999')

        if swing.back_dur_ms > 0 and swing.down_dur_ms > 0:
            ratio = swing.back_dur_ms / swing.down_dur_ms
            tempo_color = '#2ca02c' if 2.7 <= ratio <= 3.3 else '#888888'
            # Highlight angle difference (dual-plane indicator)
            angle_diff = swing.back_angle_deg - swing.down_angle_deg
            diff_color = '#2ca02c' if -8 <= angle_diff <= 0 else '#cc6600'
            ax.text(0.2, -0.18,
                    f"Tempo {ratio:.2f}:1   (back {swing.back_dur_ms}ms / down {swing.down_dur_ms}ms)",
                    ha='center', fontsize=10, color=tempo_color)
            ax.text(0.2, -0.30,
                    f"Plane shift: {angle_diff:+.1f}°  "
                    f"({'flatter downswing' if angle_diff > 0 else 'steeper downswing' if angle_diff < 0 else 'same'})",
                    ha='center', fontsize=9, color=diff_color)

    def _draw_ratios(self, swing):
        ax = self.ax_ratio
        ax.clear()
        ax.set_ylim(0, 1)
        ax.set_xlabel("Phase progress")
        ax.set_ylabel("Off-plane ratio")
        ax.set_title(f"On-plane quality  ({len(swing.back_ratios)}+{len(swing.down_ratios)} samples)")
        ax.axhline(0.25, color='gray', ls='--', lw=0.7, label='threshold')
        if len(swing.back_ratios) > 0:
            x = np.linspace(0, 1, len(swing.back_ratios))
            ax.plot(x, swing.back_ratios, color='#1f77b4', lw=1.5, label='Backswing')
        if len(swing.down_ratios) > 0:
            x = np.linspace(1.05, 2.05, len(swing.down_ratios))
            ax.plot(x, swing.down_ratios, color='#d62728', lw=1.5, label='Downswing')
        ax.set_xlim(0, 2.1)
        ax.axvspan(0, 1, alpha=0.05, color='blue')
        ax.axvspan(1.05, 2.05, alpha=0.05, color='red')
        ax.legend(loc='upper right', fontsize=9)


async def ble_loop(viewer):
    while True:
        try:
            viewer.set_status("Scanning for Nimbrace...")
            print("Scanning...")
            device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
            if device is None:
                viewer.set_status("Nimbrace not found, retrying...", '#cc6600')
                print("Not found, retrying in 3s...")
                await asyncio.sleep(3)
                continue

            print(f"Found {device.name} [{device.address}], connecting...")
            viewer.set_status(f"Found {device.name}, connecting...", '#666')

            disconnected_evt = asyncio.Event()

            def on_disconnect(_client):
                ts = time.strftime("%H:%M:%S")
                print(f"!!! [{ts}] Disconnected")
                disconnected_evt.set()

            def on_swing(s):
                print(f"  Swing: back={s.back_angle_deg:.1f}° down={s.down_angle_deg:.1f}° "
                      f"({s.back_dur_ms}ms / {s.down_dur_ms}ms, "
                      f"{len(s.back_ratios)}+{len(s.down_ratios)} samples)")
                viewer.update(s)

            parser = PacketParser(on_swing, viewer.heartbeat)

            def on_rx(_, data):
                print(f"  RX {len(data)} bytes: {bytes(data).hex()}")
                parser.feed(bytes(data))

            async with BleakClient(device, disconnected_callback=on_disconnect) as client:
                await client.start_notify(NUS_TX_UUID, on_rx)
                viewer.set_status(f"Connected to {device.name} — swing!", '#2ca02c')
                print(f"Connected. MTU={getattr(client, 'mtu_size', '?')}. Waiting for swings...")
                await disconnected_evt.wait()
                viewer.set_status("Disconnected, reconnecting...", '#cc6600')

        except Exception as e:
            viewer.set_status(f"BLE error: {e}", '#cc0000')
            print(f"BLE error: {e}")
            await asyncio.sleep(3)


async def main():
    viewer = Viewer()
    plt.show(block=False)

    ble_task = asyncio.create_task(ble_loop(viewer))

    try:
        while True:
            viewer.fig.canvas.flush_events()
            await asyncio.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        ble_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass

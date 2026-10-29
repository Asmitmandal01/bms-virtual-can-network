"""
Virtual CAN 2.0B Bus Implementation with Arbitration, Subscriptions,
Vector CANalyzer (.asc) Trace Generation, and CSV Frame Logging.
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional
import csv
import struct


# Standard CAN Arbitration IDs (11-bit Standard IDs, ordered by priority)
CAN_ID_BMS_FAULT_REPORT   = 0x080  # Highest priority emergency fault frame (BMS -> Dashboard)
CAN_ID_BATTERY_PACK_VI    = 0x101  # Pack Voltage & Current (Sensor -> BMS, Dashboard)
CAN_ID_CELL_VOLTAGES      = 0x102  # Cell 1..4 Voltages in mV (Sensor -> BMS, Dashboard)
CAN_ID_TEMPERATURES       = 0x103  # Module 1, Module 2, Ambient Temps (Sensor -> BMS, Dashboard)
CAN_ID_BMS_STATUS_SOC     = 0x201  # BMS State, SOC, SOH, Current Limits, Contactors (BMS -> Dashboard)
CAN_ID_BMS_CELL_BALANCING = 0x202  # Cell Balancing Shunt Mask & Delta-V (BMS -> Sensor, Dashboard)
CAN_ID_DASHBOARD_CMD      = 0x301  # Driver Mode Request & Fault Reset (Dashboard -> BMS)

CAN_ID_NAMES = {
    CAN_ID_BMS_FAULT_REPORT:   "BMS_Fault_Report",
    CAN_ID_BATTERY_PACK_VI:    "Battery_Pack_VI",
    CAN_ID_CELL_VOLTAGES:      "Battery_Cell_Voltages",
    CAN_ID_TEMPERATURES:       "Battery_Temperatures",
    CAN_ID_BMS_STATUS_SOC:     "BMS_Status_SOC",
    CAN_ID_BMS_CELL_BALANCING: "BMS_Cell_Balancing",
    CAN_ID_DASHBOARD_CMD:      "Dashboard_Command",
}


def compute_crc8(data: bytes) -> int:
    """SAE J1850 CRC-8 (polynomial 0x1D, init 0xFF) for automotive CAN E2E protection."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


@dataclass
class CANFrame:
    """Represents a single CAN 2.0B frame on the virtual bus."""
    arbitration_id: int
    data: bytes
    timestamp: float = 0.0
    sender: str = "UNKNOWN"
    channel: int = 1
    decoded_summary: str = ""

    @property
    def dlc(self) -> int:
        return len(self.data)

    @property
    def id_hex(self) -> str:
        return f"0x{self.arbitration_id:03X}"

    @property
    def msg_name(self) -> str:
        return CAN_ID_NAMES.get(self.arbitration_id, f"Unknown_{self.id_hex}")

    @property
    def data_hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.data)


class VirtualCANBus:
    """
    Deterministic in-memory CAN 2.0B bus simulating bitwise arbitration priority
    (lower CAN ID wins arbitration when queued simultaneously) and full trace logging.
    """

    def __init__(self, bus_name: str = "vcan0", bitrate: int = 500_000):
        self.bus_name = bus_name
        self.bitrate = bitrate
        self.subscribers: Dict[int, List[Callable[[CANFrame], None]]] = {}
        self.wildcard_subscribers: List[Callable[[CANFrame], None]] = []
        self.tx_queue: List[CANFrame] = []
        self.trace_log: List[CANFrame] = []
        self.total_frames_transmitted: int = 0
        self.bus_load_bytes: int = 0

    def subscribe(self, can_id: Optional[int], callback: Callable[[CANFrame], None]) -> None:
        """Register an ECU receive callback for a specific CAN ID or all frames (can_id=None)."""
        if can_id is None:
            self.wildcard_subscribers.append(callback)
        else:
            self.subscribers.setdefault(can_id, []).append(callback)

    def send(self, frame: CANFrame) -> None:
        """Queue a CAN frame for transmission on the bus."""
        if len(frame.data) > 8:
            raise ValueError(f"Classic CAN 2.0B frame payload cannot exceed 8 bytes (got {len(frame.data)})")
        self.tx_queue.append(frame)

    def flush(self) -> int:
        """
        Arbitrate queued frames by (timestamp, arbitration_id) so higher-priority
        frames (e.g., 0x080 Fault Report) win the bus first, then deliver to all ECUs.
        """
        if not self.tx_queue:
            return 0

        # Sort by timestamp first, then by lowest arbitration_id (CAN dominant bit priority)
        self.tx_queue.sort(key=lambda f: (round(f.timestamp, 6), f.arbitration_id))
        delivered = 0

        while self.tx_queue:
            frame = self.tx_queue.pop(0)
            self.trace_log.append(frame)
            self.total_frames_transmitted += 1
            # Approximate wire size: 47 bits overhead + 8 * DLC bits
            self.bus_load_bytes += (6 + frame.dlc)

            for cb in self.subscribers.get(frame.arbitration_id, []):
                cb(frame)
            for cb in self.wildcard_subscribers:
                cb(frame)
            delivered += 1

        return delivered

    def export_asc_log(self, filepath: str, max_frames: Optional[int] = None) -> None:
        """Export recorded bus traffic in industry-standard Vector CANalyzer (.asc) format."""
        frames = self.trace_log if max_frames is None else self.trace_log[:max_frames]
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("date Thu Oct 29 22:00:00.000 2026\n")
            f.write("base hex  timestamps absolute\n")
            f.write("internal events logged\n")
            f.write("// version 11.0.0 - Virtual CAN BMS Bus Trace (vcan0 @ 500 kbps)\n")
            f.write("Begin Triggerblock Thu Oct 29 22:00:00.000 2026\n")
            f.write("   0.000000 Start of measurement\n")
            for fr in frames:
                f.write(
                    f"   {fr.timestamp:10.6f} {fr.channel}  {fr.arbitration_id:03X}x"
                    f"             Rx   d {fr.dlc} {fr.data_hex:<23}  // [{fr.sender}] {fr.msg_name}: {fr.decoded_summary}\n"
                )
            f.write("End TriggerBlock\n")

    def export_csv_log(self, filepath: str, max_frames: Optional[int] = None) -> None:
        """Export recorded bus traffic to CSV with decoded engineering summaries."""
        frames = self.trace_log if max_frames is None else self.trace_log[:max_frames]
        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Timestamp_s", "Channel", "CAN_ID_Hex", "Message_Name",
                "Sender_ECU", "DLC", "Payload_Hex", "Decoded_Engineering_Values"
            ])
            for fr in frames:
                writer.writerow([
                    f"{fr.timestamp:.4f}",
                    self.bus_name,
                    fr.id_hex,
                    fr.msg_name,
                    fr.sender,
                    fr.dlc,
                    fr.data_hex,
                    fr.decoded_summary,
                ])

"""
ECU 3: Instrument Cluster & Supervisory Dashboard Node
Responsibilities:
  1. Subscribe to BMS_Status_SOC (0x201), BMS_Fault_Report (0x080), and raw Sensor frames.
  2. Maintain driver-visible warning indicators (MIL lamp, HV Contactor status, Fault DTC banner).
  3. Record synchronized time-series telemetry for post-run engineering evaluation.
  4. Transmit periodic 0x301 Dashboard_Command frames (10 Hz / 100 ms).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import struct

from .can_bus import (
    CANFrame,
    VirtualCANBus,
    CAN_ID_BMS_FAULT_REPORT,
    CAN_ID_BATTERY_PACK_VI,
    CAN_ID_CELL_VOLTAGES,
    CAN_ID_TEMPERATURES,
    CAN_ID_BMS_STATUS_SOC,
    CAN_ID_BMS_CELL_BALANCING,
    CAN_ID_DASHBOARD_CMD,
)
from .bms_controller import BMSState


@dataclass
class DashboardAlertEvent:
    """Represents a fault alert received and displayed by the Dashboard ECU."""
    timestamp_s: float
    fault_flags: int
    dtc_code: int
    cell_or_mod_index: int
    trigger_value: int
    message: str


class DashboardNodeECU:
    """Simulated Instrument Cluster / Dashboard ECU on the Virtual CAN Bus."""

    def __init__(self, bus: VirtualCANBus):
        self.name = "Dashboard_Node"
        self.bus = bus

        # Operator controls
        self.drive_enable_req: bool = True
        self.clear_faults_req: bool = False
        self.operating_mode_req: int = 1
        self.alive_301: int = 0
        self._last_tx_301: float = -1.0

        # Displayed cluster values
        self.displayed_soc_pct: float = 0.0
        self.displayed_soh_pct: float = 100.0
        self.displayed_bms_state: str = "INIT"
        self.displayed_pack_v: float = 0.0
        self.displayed_pack_i: float = 0.0
        self.displayed_cell_mv: List[int] = [0, 0, 0, 0]
        self.displayed_temps_c: List[float] = [25.0, 25.0, 25.0]
        self.hv_contactor_closed: bool = False
        self.balancing_active: bool = False
        self.mil_fault_lamp_on: bool = False
        self.active_dtc_hex: str = "NONE"

        # Logged fault events
        self.fault_events: List[DashboardAlertEvent] = []

        # Subscribe to CAN bus frames
        self.bus.subscribe(CAN_ID_BMS_FAULT_REPORT, self._on_fault_report)
        self.bus.subscribe(CAN_ID_BMS_STATUS_SOC, self._on_bms_status)
        self.bus.subscribe(CAN_ID_BATTERY_PACK_VI, self._on_pack_vi)
        self.bus.subscribe(CAN_ID_CELL_VOLTAGES, self._on_cell_voltages)
        self.bus.subscribe(CAN_ID_TEMPERATURES, self._on_temperatures)

    def _on_fault_report(self, frame: CANFrame) -> None:
        if len(frame.data) < 8:
            return
        flags, dtc, sev_reason, idx, trig_val, _ = struct.unpack("<BHBBhB", frame.data[:8])
        self.mil_fault_lamp_on = True
        self.hv_contactor_closed = False
        self.active_dtc_hex = f"0x{dtc:04X}"

        msg = (
            f"FAULT ALERT [DTC 0x{dtc:04X}]: Flags=0x{flags:02X}, "
            f"SensorIdx={idx}, Val={trig_val} -> HV CONTACTORS OPENED"
        )
        if not self.fault_events or self.fault_events[-1].dtc_code != dtc:
            self.fault_events.append(DashboardAlertEvent(
                timestamp_s=frame.timestamp,
                fault_flags=flags,
                dtc_code=dtc,
                cell_or_mod_index=idx,
                trigger_value=trig_val,
                message=msg,
            ))

    def _on_bms_status(self, frame: CANFrame) -> None:
        if len(frame.data) < 8:
            return
        state_val, soc_d1, soh_d1, dis_lim, chg_lim, flags = struct.unpack("<BHHBBB", frame.data[:8])
        try:
            self.displayed_bms_state = BMSState(state_val).name
        except ValueError:
            self.displayed_bms_state = f"UNKNOWN({state_val})"
        self.displayed_soc_pct = soc_d1 / 10.0
        self.displayed_soh_pct = soh_d1 / 10.0
        self.hv_contactor_closed = bool(flags & 0x01)
        self.balancing_active = bool(flags & 0x02)

    def _on_pack_vi(self, frame: CANFrame) -> None:
        if len(frame.data) >= 4:
            pack_mv, curr_da = struct.unpack("<Hh", frame.data[:4])
            self.displayed_pack_v = pack_mv / 1000.0
            self.displayed_pack_i = curr_da / 10.0

    def _on_cell_voltages(self, frame: CANFrame) -> None:
        if len(frame.data) >= 8:
            self.displayed_cell_mv = list(struct.unpack("<HHHH", frame.data[:8]))

    def _on_temperatures(self, frame: CANFrame) -> None:
        if len(frame.data) >= 6:
            t1, t2, tamb = struct.unpack("<hhh", frame.data[:6])
            self.displayed_temps_c = [t1 / 10.0, t2 / 10.0, tamb / 10.0]

    def step(self, timestamp_s: float) -> None:
        """Transmit periodic 100 ms Dashboard_Command frame (0x301)."""
        if timestamp_s - self._last_tx_301 >= 0.0999:
            cmd_flags = (0x01 if self.drive_enable_req else 0x00) | (0x02 if self.clear_faults_req else 0x00)
            payload = struct.pack(
                "<BBBBBBBB",
                cmd_flags,
                self.operating_mode_req,
                self.alive_301,
                0, 0, 0, 0, 0,
            )
            summary = (
                f"DriveEnable={int(self.drive_enable_req)}, "
                f"ClearFault={int(self.clear_faults_req)}, ModeReq={self.operating_mode_req}"
            )
            self.bus.send(CANFrame(
                arbitration_id=CAN_ID_DASHBOARD_CMD,
                data=payload,
                timestamp=timestamp_s,
                sender=self.name,
                decoded_summary=summary,
            ))
            self.alive_301 = (self.alive_301 + 1) & 0xFF
            self.clear_faults_req = False
            self._last_tx_301 = timestamp_s

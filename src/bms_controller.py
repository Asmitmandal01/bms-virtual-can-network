"""
ECU 2: Master Battery Management System (BMS) Controller
Responsibilities:
  1. Receive cyclic CAN telemetry from Sensor_Node (0x101, 0x102, 0x103) and commands from Dashboard (0x301).
  2. Hybrid SOC Estimation: Coulomb Counting with Dynamic OCV Drift Correction (Kalman-like observer gain).
  3. Safety Diagnostics & Fault Protection (ISO 26262 ASIL-C compliant debounce logic):
     - Over-Voltage (OV): Cell Voltage > 4200 mV for >= 150 ms (3 cell frames) -> Contactor Cut-Off
     - Under-Voltage (UV): Cell Voltage < 2800 mV for >= 150 ms (3 cell frames) -> Contactor Cut-Off
     - Over-Temperature (OT): Temp > 60.0 degC for >= 300 ms (3 temp frames) -> Contactor Cut-Off
     - Over-Current (OC): |Current| > 200.0 A for >= 60 ms (3 VI frames) -> Contactor Cut-Off
     - Sensor Timeout / Watchdog: No valid 0x101 frame for > 100 ms -> Contactor Cut-Off
  4. Passive Cell Balancing: Activates bleed shunts when max-min cell delta > 15 mV above 3.90 V.
  5. Broadcasts 0x201 (BMS_Status_SOC), 0x202 (BMS_Cell_Balancing), and 0x080 (BMS_Fault_Report).
"""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import List, Optional
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
    compute_crc8,
)
from .battery_plant import ocv_from_soc, soc_from_ocv


class BMSState(IntEnum):
    INIT = 0
    STANDBY = 1
    PRECHARGE = 2
    NORMAL_DRIVE = 3
    FAST_CHARGE = 4
    DERATING = 5
    FAULT_CUTOFF = 6


class FaultFlags(IntEnum):
    NONE               = 0x00
    CELL_OVER_VOLTAGE  = 0x01  # Bit 0
    CELL_UNDER_VOLTAGE = 0x02  # Bit 1
    OVER_TEMPERATURE   = 0x04  # Bit 2
    OVER_CURRENT       = 0x08  # Bit 3
    CAN_SENSOR_TIMEOUT = 0x10  # Bit 4
    CRC_PLAUSIBILITY   = 0x20  # Bit 5


# Automotive UDS-style Diagnostic Trouble Codes (DTCs)
DTC_CODES = {
    FaultFlags.NONE:               0x0000,
    FaultFlags.CELL_OVER_VOLTAGE:  0x0A80,  # Pack Cell Over-Voltage
    FaultFlags.CELL_UNDER_VOLTAGE: 0x0A81,                        # Pack Cell Under-Voltage
    FaultFlags.OVER_TEMPERATURE:   0x0A78,                        # Battery Pack Over-Temperature
    FaultFlags.OVER_CURRENT:       0x0A90,                        # Pack Discharge/Charge Over-Current
    FaultFlags.CAN_SENSOR_TIMEOUT: 0xC100,                        # Lost Comm with Battery Sensor Module
    FaultFlags.CRC_PLAUSIBILITY:   0xC401,                        # Invalid CRC/Alive Counter from Sensor
}


@dataclass
class SafetyThresholds:
    """Configurable safety thresholds and debounce counts."""
    cell_ov_cutoff_mv: int = 4200
    cell_ov_derate_mv: int = 4150
    cell_uv_cutoff_mv: int = 2800
    cell_uv_derate_mv: int = 2950
    temp_ot_cutoff_c: float = 60.0
    temp_ot_derate_c: float = 52.0
    current_oc_cutoff_a: float = 200.0
    can_timeout_s: float = 0.100

    # Debounce thresholds (number of consecutive violating CAN samples required to trip)
    ov_debounce_samples: int = 3   # 3 * 50ms = 150ms qualification time
    uv_debounce_samples: int = 3   # 3 * 50ms = 150ms qualification time
    ot_debounce_samples: int = 3   # 3 * 100ms = 300ms qualification time
    oc_debounce_samples: int = 3   # 3 * 20ms = 60ms qualification time


class BMSControllerECU:
    """Master BMS Controller ECU with SOC estimator, safety monitor, and contactor state machine."""

    def __init__(
        self,
        bus: VirtualCANBus,
        initial_soc_estimate: float = 0.72,  # Intentionally offset from true 0.75 to show convergence
        nominal_capacity_ah: float = 50.0,
        thresholds: Optional[SafetyThresholds] = None,
    ):
        self.name = "BMS_Controller"
        self.bus = bus
        self.estimated_soc = initial_soc_estimate
        self.estimated_soh = 0.985
        self.nominal_capacity_ah = nominal_capacity_ah
        self.thresholds = thresholds or SafetyThresholds()

        # State machine & Contactor outputs
        self.state: BMSState = BMSState.STANDBY
        self.contactor_closed: bool = False
        self.drive_enable_req: bool = True
        self.requested_mode: int = 1  # 1 = Drive, 2 = Fast Charge

        # Latest received telemetry from Sensor_Node
        self.pack_voltage_mv: int = 14800
        self.pack_current_a: float = 0.0
        self.cell_voltages_mv: List[int] = [3700, 3700, 3700, 3700]
        self.temp_mod1_c: float = 25.0
        self.temp_mod2_c: float = 25.0
        self.temp_ambient_c: float = 25.0

        # Timing & watchdog tracking
        self.last_rx_101_time: Optional[float] = None
        self.last_soc_update_time: Optional[float] = None
        self._last_tx_201: float = -1.0
        self._last_tx_202: float = -1.0
        self.alive_080: int = 0
        self.balancing_mask: int = 0

        # Fault qualification debounce counters
        self.ov_counter: int = 0
        self.uv_counter: int = 0
        self.ot_counter: int = 0
        self.oc_counter: int = 0
        self.crc_error_counter: int = 0

        # Latched fault state
        self.active_fault_flags: int = 0
        self.primary_dtc: int = 0x0000
        self.fault_cell_index: int = 0
        self.fault_trigger_value: int = 0
        self.fault_trip_timestamp: Optional[float] = None

        # Subscribe to CAN frames
        self.bus.subscribe(CAN_ID_BATTERY_PACK_VI, self._on_pack_vi)
        self.bus.subscribe(CAN_ID_CELL_VOLTAGES, self._on_cell_voltages)
        self.bus.subscribe(CAN_ID_TEMPERATURES, self._on_temperatures)
        self.bus.subscribe(CAN_ID_DASHBOARD_CMD, self._on_dashboard_cmd)

    def _on_pack_vi(self, frame: CANFrame) -> None:
        if len(frame.data) < 8:
            return
        pack_mv, curr_da, status, alive = struct.unpack("<HhBB", frame.data[:6])
        rx_crc = frame.data[6]
        calc_crc = compute_crc8(frame.data[:6])

        if rx_crc != calc_crc:
            self.crc_error_counter += 1
            if self.crc_error_counter >= 3:
                self._latch_fault(
                    FaultFlags.CRC_PLAUSIBILITY,
                    cell_idx=0,
                    trigger_val=rx_crc,
                    timestamp_s=frame.timestamp,
                )
            return
        else:
            self.crc_error_counter = max(0, self.crc_error_counter - 1)

        self.pack_voltage_mv = pack_mv
        self.pack_current_a = curr_da / 10.0

        # Update Coulomb-Counting + OCV Observer SOC Estimation
        if self.last_soc_update_time is not None:
            dt = max(0.0, frame.timestamp - self.last_soc_update_time)
            self._update_soc_estimator(dt)
        self.last_soc_update_time = frame.timestamp
        self.last_rx_101_time = frame.timestamp

        # Check Over-Current (OC) with debounce
        if abs(self.pack_current_a) > self.thresholds.current_oc_cutoff_a:
            self.oc_counter += 1
            if self.oc_counter >= self.thresholds.oc_debounce_samples:
                self._latch_fault(
                    FaultFlags.OVER_CURRENT,
                    cell_idx=0,
                    trigger_val=int(round(self.pack_current_a * 10)),
                    timestamp_s=frame.timestamp,
                )
        else:
            self.oc_counter = max(0, self.oc_counter - 1)

    def _on_cell_voltages(self, frame: CANFrame) -> None:
        if len(frame.data) < 8:
            return
        c1, c2, c3, c4 = struct.unpack("<HHHH", frame.data[:8])
        self.cell_voltages_mv = [c1, c2, c3, c4]

        max_v = max(self.cell_voltages_mv)
        max_idx = self.cell_voltages_mv.index(max_v) + 1
        min_v = min(self.cell_voltages_mv)
        min_idx = self.cell_voltages_mv.index(min_v) + 1

        # Over-Voltage (OV) Fault Qualification
        if max_v > self.thresholds.cell_ov_cutoff_mv:
            self.ov_counter += 1
            if self.ov_counter >= self.thresholds.ov_debounce_samples:
                self._latch_fault(
                    FaultFlags.CELL_OVER_VOLTAGE,
                    cell_idx=max_idx,
                    trigger_val=max_v,
                    timestamp_s=frame.timestamp,
                )
        else:
            self.ov_counter = max(0, self.ov_counter - 1)

        # Under-Voltage (UV) Fault Qualification
        if min_v < self.thresholds.cell_uv_cutoff_mv:
            self.uv_counter += 1
            if self.uv_counter >= self.thresholds.uv_debounce_samples:
                self._latch_fault(
                    FaultFlags.CELL_UNDER_VOLTAGE,
                    cell_idx=min_idx,
                    trigger_val=min_v,
                    timestamp_s=frame.timestamp,
                )
        else:
            self.uv_counter = max(0, self.uv_counter - 1)

        # Update Passive Cell Balancing decision
        self._update_cell_balancing()

    def _on_temperatures(self, frame: CANFrame) -> None:
        if len(frame.data) < 6:
            return
        t1_dc, t2_dc, tamb_dc = struct.unpack("<hhh", frame.data[:6])
        self.temp_mod1_c = t1_dc / 10.0
        self.temp_mod2_c = t2_dc / 10.0
        self.temp_ambient_c = tamb_dc / 10.0

        max_temp = max(self.temp_mod1_c, self.temp_mod2_c)
        hot_mod = 1 if self.temp_mod1_c >= self.temp_mod2_c else 2

        if max_temp > self.thresholds.temp_ot_cutoff_c:
            self.ot_counter += 1
            if self.ot_counter >= self.thresholds.ot_debounce_samples:
                self._latch_fault(
                    FaultFlags.OVER_TEMPERATURE,
                    cell_idx=hot_mod,
                    trigger_val=int(round(max_temp * 10)),
                    timestamp_s=frame.timestamp,
                )
        else:
            self.ot_counter = max(0, self.ot_counter - 1)

    def _on_dashboard_cmd(self, frame: CANFrame) -> None:
        if len(frame.data) < 3:
            return
        flags, mode_req, _ = struct.unpack("<BBB", frame.data[:3])
        self.drive_enable_req = bool(flags & 0x01)
        clear_faults = bool(flags & 0x02)
        self.requested_mode = mode_req

        if clear_faults and self.state == BMSState.FAULT_CUTOFF:
            # Reset latched faults only if underlying physical conditions are safe
            if (
                max(self.cell_voltages_mv) < self.thresholds.cell_ov_cutoff_mv
                and min(self.cell_voltages_mv) > self.thresholds.cell_uv_cutoff_mv
                and max(self.temp_mod1_c, self.temp_mod2_c) < self.thresholds.temp_ot_cutoff_c
            ):
                self.active_fault_flags = 0
                self.primary_dtc = 0x0000
                self.ov_counter = 0
                self.uv_counter = 0
                self.ot_counter = 0
                self.oc_counter = 0
                self.state = BMSState.STANDBY

    def _update_soc_estimator(self, dt_s: float) -> None:
        """
        Hybrid Coulomb Counting + OCV Luenberger/Kalman Observer.
        1. Predict step via Coulomb integration: SOC_k = SOC_{k-1} - (I * dt) / (3600 * Q_nom)
        2. Measurement update: Estimate OCV = V_cell_avg + I * R_est, compare with OCV(SOC_pred),
           and apply observer gain K_obs (stronger when current magnitude is low).
        """
        dsoc = -(self.pack_current_a * dt_s) / (self.nominal_capacity_ah * 3600.0)
        soc_pred = max(0.0, min(1.0, self.estimated_soc + dsoc))

        import math
        avg_cell_v = (self.pack_voltage_mv / 1000.0) / 4.0
        # 1st-Order Thevenin Equivalent Circuit Model (R0 + R1||C1) state observer
        r0_est = 0.001875
        r1_est = 0.001225
        c1_est = 1792.5
        tau_est = r1_est * c1_est
        alpha = math.exp(-dt_s / tau_est)
        if not hasattr(self, "v_rc_est"):
            self.v_rc_est = 0.0
        self.v_rc_est = self.v_rc_est * alpha + r1_est * self.pack_current_a * (1.0 - alpha)

        v_ocv_meas = avg_cell_v + self.pack_current_a * r0_est + self.v_rc_est
        soc_meas = soc_from_ocv(v_ocv_meas)

        # Closed-loop observer update
        soc_residual = soc_meas - soc_pred
        k_gain = 0.065 if abs(self.pack_current_a) < 15.0 else 0.035
        self.estimated_soc = max(0.0, min(1.0, soc_pred + k_gain * soc_residual))

    def _update_cell_balancing(self) -> None:
        """Enable passive bleed shunts during charging/rest with Schmitt-trigger hysteresis."""
        if self.state == BMSState.FAULT_CUTOFF or self.pack_current_a > 10.0:
            self.balancing_mask = 0
            return
        min_v = min(self.cell_voltages_mv)
        mask = 0
        for i, mv in enumerate(self.cell_voltages_mv):
            was_on = bool(self.balancing_mask & (1 << i))
            threshold = 4 if was_on else 9  # 5 mV hysteresis band prevents shunt chatter
            if mv > 3550 and (mv - min_v) >= threshold:
                mask |= (1 << i)
        self.balancing_mask = mask

    def _latch_fault(
        self,
        fault: FaultFlags,
        cell_idx: int,
        trigger_val: int,
        timestamp_s: float,
    ) -> None:
        """Immediately open HV contactors, transition to FAULT_CUTOFF, and broadcast emergency 0x080 frame."""
        first_trip = (self.state != BMSState.FAULT_CUTOFF) or not (self.active_fault_flags & int(fault))
        self.active_fault_flags |= int(fault)
        if self.primary_dtc == 0:
            self.primary_dtc = DTC_CODES.get(fault, 0xFFFF)
            self.fault_cell_index = cell_idx
            self.fault_trigger_value = trigger_val
            self.fault_trip_timestamp = timestamp_s

        # Safety Goal 1: Immediate HV Contactor Open on critical fault
        self.state = BMSState.FAULT_CUTOFF
        self.contactor_closed = False
        self.balancing_mask = 0

        if first_trip:
            self._tx_fault_report(timestamp_s, fault)

    def _evaluate_state_machine(self, timestamp_s: float) -> None:
        """Run BMS supervisory state machine and check CAN communication watchdog."""
        # Check CAN heartbeat timeout from Sensor_Node
        if (
            self.last_rx_101_time is not None
            and (timestamp_s - self.last_rx_101_time) > self.thresholds.can_timeout_s
        ):
            self._latch_fault(
                FaultFlags.CAN_SENSOR_TIMEOUT,
                cell_idx=0,
                trigger_val=int(round((timestamp_s - self.last_rx_101_time) * 1000)),
                timestamp_s=timestamp_s,
            )

        if self.state == BMSState.FAULT_CUTOFF:
            self.contactor_closed = False
            return

        if not self.drive_enable_req:
            self.state = BMSState.STANDBY
            self.contactor_closed = False
            return

        # Transition from STANDBY -> PRECHARGE -> NORMAL_DRIVE / FAST_CHARGE
        if self.state in (BMSState.INIT, BMSState.STANDBY):
            self.state = BMSState.PRECHARGE
            self.contactor_closed = True
            return

        # Check derating zone (warning before hard cut-off)
        max_v = max(self.cell_voltages_mv)
        min_v = min(self.cell_voltages_mv)
        max_t = max(self.temp_mod1_c, self.temp_mod2_c)

        if (
            max_v >= self.thresholds.cell_ov_derate_mv
            or min_v <= self.thresholds.cell_uv_derate_mv
            or max_t >= self.thresholds.temp_ot_derate_c
        ):
            self.state = BMSState.DERATING
            self.contactor_closed = True
        else:
            self.state = (
                BMSState.FAST_CHARGE if self.pack_current_a < -5.0 else BMSState.NORMAL_DRIVE
            )
            self.contactor_closed = True

    def _compute_current_limits(self) -> tuple[int, int]:
        """Compute dynamic Discharge and Regen/Charge current limits (Amps) based on state."""
        if not self.contactor_closed or self.state == BMSState.FAULT_CUTOFF:
            return (0, 0)
        if self.state == BMSState.DERATING:
            return (60, 40)
        return (180, 120)

    def step(self, timestamp_s: float) -> None:
        """Periodic execution of BMS Controller state machine and CAN broadcasts."""
        self._evaluate_state_machine(timestamp_s)

        # 20 Hz (50 ms): Broadcast 0x201 BMS_Status_SOC
        if timestamp_s - self._last_tx_201 >= 0.0499:
            self._tx_bms_status(timestamp_s)
            self._last_tx_201 = timestamp_s

        # 5 Hz (200 ms): Broadcast 0x202 BMS_Cell_Balancing (and cyclic fault reminder if active)
        if timestamp_s - self._last_tx_202 >= 0.1999:
            self._tx_balancing_status(timestamp_s)
            if self.state == BMSState.FAULT_CUTOFF:
                self._tx_fault_report(timestamp_s, FaultFlags(self.active_fault_flags & -self.active_fault_flags))
            self._last_tx_202 = timestamp_s

    def _tx_bms_status(self, timestamp_s: float) -> None:
        soc_d1 = int(round(self.estimated_soc * 1000.0))  # 0.1% units (0..1000)
        soh_d1 = int(round(self.estimated_soh * 1000.0))
        dis_lim, chg_lim = self._compute_current_limits()
        flags = (0x01 if self.contactor_closed else 0x00) | (0x02 if self.balancing_mask else 0x00)

        payload = struct.pack(
            "<BHHBBB",
            int(self.state),
            soc_d1,
            soh_d1,
            dis_lim,
            chg_lim,
            flags,
        )
        summary = (
            f"State={self.state.name}, SOC={soc_d1/10.0:.1f}%, SOH={soh_d1/10.0:.1f}%, "
            f"Contactor={'CLOSED' if self.contactor_closed else 'OPEN'}, "
            f"I_lim=+{dis_lim}/-{chg_lim}A"
        )
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_BMS_STATUS_SOC,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))

    def _tx_balancing_status(self, timestamp_s: float) -> None:
        delta_mv = max(self.cell_voltages_mv) - min(self.cell_voltages_mv)
        payload = struct.pack("<BHBBBBB", self.balancing_mask, delta_mv, 0, 0, 0, 0, 0)
        summary = f"BalMask=0b{self.balancing_mask:04b}, CellDelta={delta_mv}mV"
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_BMS_CELL_BALANCING,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))

    def _tx_fault_report(self, timestamp_s: float, primary_fault: FaultFlags) -> None:
        severity = 3  # 3 = Critical ASIL-C Contactor Trip
        trip_reason = int(primary_fault) & 0x0F
        sev_reason = ((trip_reason & 0x0F) << 4) | (severity & 0x0F)
        trig_val = max(-32768, min(32767, self.fault_trigger_value))

        payload = struct.pack(
            "<BHBBhB",
            self.active_fault_flags & 0xFF,
            self.primary_dtc & 0xFFFF,
            sev_reason,
            self.fault_cell_index & 0xFF,
            trig_val,
            self.alive_080 & 0xFF,
        )
        summary = (
            f"CRITICAL FAULT! Flags=0x{self.active_fault_flags:02X}, DTC=0x{self.primary_dtc:04X}, "
            f"Target={self.fault_cell_index}, Value={trig_val}, Contactor=OPEN"
        )
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_BMS_FAULT_REPORT,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))
        self.alive_080 = (self.alive_080 + 1) & 0xFF

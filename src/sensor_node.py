"""
ECU 1: Battery Sensor Node (BMU - Battery Monitoring Unit)
Samples cell voltages, pack current, and module temperatures from the physical
battery plant, applies realistic ADC quantization/noise and optional Fault Injection
overrides, and transmits cyclic CAN frames:
  - 0x101 Battery_Pack_VI        (every 20 ms / 50 Hz)
  - 0x102 Battery_Cell_Voltages  (every 50 ms / 20 Hz)
  - 0x103 Battery_Temperatures   (every 100 ms / 10 Hz)
"""

from dataclasses import dataclass
from typing import Optional
import random
import struct

from .can_bus import (
    CANFrame,
    VirtualCANBus,
    CAN_ID_BATTERY_PACK_VI,
    CAN_ID_CELL_VOLTAGES,
    CAN_ID_TEMPERATURES,
    CAN_ID_BMS_CELL_BALANCING,
    compute_crc8,
)
from .battery_plant import BatteryPackPlant


@dataclass
class FaultInjectionConfig:
    """Configures deliberate fault injection in the Sensor Node for verification testing."""
    override_cell_index: Optional[int] = None      # 0..3
    override_cell_voltage_mv: Optional[int] = None # e.g. 4280 (OV) or 2690 (UV)
    override_temp_mod2_c: Optional[float] = None   # e.g. 64.5 (OT)
    override_pack_current_a: Optional[float] = None # e.g. 240.0 (OC)
    mute_can_transmission: bool = False            # Simulates broken CAN wire / dead sensor ECU
    corrupt_crc: bool = False                      # Simulates E2E CRC corruption


class BatterySensorNodeECU:
    """Simulated Battery Sensor Node ECU connected to the Virtual CAN Bus."""

    def __init__(self, bus: VirtualCANBus, plant: BatteryPackPlant, seed: int = 42):
        self.name = "Sensor_Node"
        self.bus = bus
        self.plant = plant
        self.rng = random.Random(seed)
        self.fault_injection = FaultInjectionConfig()

        self.alive_101 = 0
        self.alive_103 = 0
        self._last_tx_101 = -1.0
        self._last_tx_102 = -1.0
        self._last_tx_103 = -1.0

        # Subscribe to balancing commands from BMS Controller
        self.bus.subscribe(CAN_ID_BMS_CELL_BALANCING, self._on_balancing_cmd)

    def _on_balancing_cmd(self, frame: CANFrame) -> None:
        if len(frame.data) >= 1:
            self.plant.balancing_mask = frame.data[0] & 0x0F

    def step(self, timestamp_s: float) -> None:
        """Execute periodic CAN transmissions according to schedule."""
        if self.fault_injection.mute_can_transmission:
            return

        # 50 Hz (20 ms): Pack Voltage & Current (0x101)
        if timestamp_s - self._last_tx_101 >= 0.0199:
            self._tx_pack_vi(timestamp_s)
            self._last_tx_101 = timestamp_s

        # 20 Hz (50 ms): Individual Cell Voltages (0x102)
        if timestamp_s - self._last_tx_102 >= 0.0499:
            self._tx_cell_voltages(timestamp_s)
            self._last_tx_102 = timestamp_s

        # 10 Hz (100 ms): Module Temperatures (0x103)
        if timestamp_s - self._last_tx_103 >= 0.0999:
            self._tx_temperatures(timestamp_s)
            self._last_tx_103 = timestamp_s

    def _get_sampled_cell_mv(self) -> list[int]:
        cell_mv = [
            int(round(v * 1000.0 + self.rng.gauss(0.0, 1.2)))
            for v in self.plant.cell_voltages_v
        ]
        if (
            self.fault_injection.override_cell_index is not None
            and self.fault_injection.override_cell_voltage_mv is not None
        ):
            cell_mv[self.fault_injection.override_cell_index] = (
                self.fault_injection.override_cell_voltage_mv
            )
        return [max(0, min(5000, mv)) for mv in cell_mv]

    def _tx_pack_vi(self, timestamp_s: float) -> None:
        cell_mv = self._get_sampled_cell_mv()
        pack_mv = sum(cell_mv)

        curr_a = self.plant.pack_current_a + self.rng.gauss(0.0, 0.08)
        if self.fault_injection.override_pack_current_a is not None:
            curr_a = self.fault_injection.override_pack_current_a

        curr_da = int(round(curr_a * 10.0))  # 0.1 A resolution, signed
        curr_da = max(-32768, min(32767, curr_da))

        sensor_status = 0x01  # 0x01 = ADC OK
        header_bytes = struct.pack("<HhBB", pack_mv, curr_da, sensor_status, self.alive_101)
        crc = compute_crc8(header_bytes)
        if self.fault_injection.corrupt_crc:
            crc ^= 0xFF
        payload = header_bytes + struct.pack("<BB", crc, 0x00)

        summary = f"V_pack={pack_mv/1000.0:.3f}V, I_pack={curr_da/10.0:+.1f}A, Alive={self.alive_101}, CRC=0x{crc:02X}"
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_BATTERY_PACK_VI,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))
        self.alive_101 = (self.alive_101 + 1) & 0xFF

    def _tx_cell_voltages(self, timestamp_s: float) -> None:
        c = self._get_sampled_cell_mv()
        payload = struct.pack("<HHHH", c[0], c[1], c[2], c[3])
        summary = f"C1={c[0]}mV, C2={c[1]}mV, C3={c[2]}mV, C4={c[3]}mV"
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_CELL_VOLTAGES,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))

    def _tx_temperatures(self, timestamp_s: float) -> None:
        t1 = self.plant.temp_mod1_c + self.rng.gauss(0.0, 0.05)
        t2 = self.plant.temp_mod2_c + self.rng.gauss(0.0, 0.05)
        if self.fault_injection.override_temp_mod2_c is not None:
            t2 = self.fault_injection.override_temp_mod2_c
        tamb = self.plant.ambient_temp_c

        t1_dc = int(round(t1 * 10.0))
        t2_dc = int(round(t2 * 10.0))
        tamb_dc = int(round(tamb * 10.0))

        payload = struct.pack("<hhhBB", t1_dc, t2_dc, tamb_dc, self.alive_103, 0x00)
        summary = f"T_mod1={t1_dc/10.0:.1f}C, T_mod2={t2_dc/10.0:.1f}C, T_amb={tamb_dc/10.0:.1f}C"
        self.bus.send(CANFrame(
            arbitration_id=CAN_ID_TEMPERATURES,
            data=payload,
            timestamp=timestamp_s,
            sender=self.name,
            decoded_summary=summary,
        ))
        self.alive_103 = (self.alive_103 + 1) & 0xFF

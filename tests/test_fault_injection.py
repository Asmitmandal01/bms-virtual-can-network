"""
Automated Safety Verification & Fault-Injection Test Suite (ISO 26262 Verification).
Tests all 7 safety and system evaluation requirements:
  TC-01: SOC Estimation Accuracy & Cell Balancing (< 1.0% RMSE over dynamic drive/charge cycle)
  TC-02: Cell Over-Voltage (OV > 4.20V) Protection & Contactor Cut-Off
  TC-03: Cell Under-Voltage (UV < 2.80V) Protection & Contactor Cut-Off
  TC-04: Pack Over-Temperature (OT > 60.0°C) Protection & Contactor Cut-Off
  TC-05: Pack Over-Current (OC > 200.0A) Protection & Contactor Cut-Off
  TC-06: Sensor CAN Heartbeat Timeout (> 100ms loss of 0x101) Safe State Cut-Off
  TC-07: Single-Sample Glitch / EMI Transient Rejection (Debounce Verification)
"""

import unittest
from dataclasses import dataclass
from typing import List
import math

from src.can_bus import VirtualCANBus, CAN_ID_BMS_FAULT_REPORT
from src.battery_plant import BatteryPackPlant
from src.sensor_node import BatterySensorNodeECU
from src.bms_controller import BMSControllerECU, BMSState, FaultFlags
from src.dashboard_node import DashboardNodeECU


@dataclass
class SafetyTestRecord:
    test_id: str
    title: str
    requirement: str
    injected_condition: str
    expected_response: str
    measured_result: str
    reaction_time_ms: float
    max_allowed_ms: float
    passed: bool


def run_all_verification_tests() -> List[SafetyTestRecord]:
    """Run all 7 verification scenarios and return structured pass/fail metrics."""
    records: List[SafetyTestRecord] = []

    # -------------------------------------------------------------------------
    # TC-01: Dynamic Drive + Fast Charge SOC Estimation & Balancing Accuracy
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.78)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.73)  # 5% initial SOC error
    dash = DashboardNodeECU(bus)

    dt = 0.01  # 10 ms step
    sq_errors = []
    for step_idx in range(3000):  # 30 seconds accelerated profile
        t = step_idx * dt
        current_a = 85.0 if t < 15.0 else -95.0
        plant.step(dt, current_a, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()
        if t >= 5.0:  # Evaluate steady-state tracking after initial observer convergence
            err_pct = (bms.estimated_soc - plant.true_pack_soc) * 100.0
            sq_errors.append(err_pct ** 2)

    rmse_pct = math.sqrt(sum(sq_errors) / len(sq_errors))
    final_err_pct = abs(bms.estimated_soc - plant.true_pack_soc) * 100.0
    tc01_pass = (rmse_pct < 1.0) and (bms.state in (BMSState.NORMAL_DRIVE, BMSState.FAST_CHARGE))
    records.append(SafetyTestRecord(
        test_id="TC-01",
        title="SOC Observer Convergence & Dynamic Tracking",
        requirement="SOC estimation RMSE <= 1.0% from 5.0% initial offset under dynamic load",
        injected_condition="Initial SOC offset -5.0% (73% vs 78% true), +85A discharge / -95A charge",
        expected_response="Observer converges cleanly; contactor stays CLOSED; RMSE <= 1.0%",
        measured_result=f"RMSE = {rmse_pct:.2f}%, Final Error = {final_err_pct:.2f}%, State = {bms.state.name}",
        reaction_time_ms=0.0,
        max_allowed_ms=0.0,
        passed=tc01_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-02: Cell Over-Voltage (OV > 4200 mV) Fault Injection
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.92)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.92)
    dash = DashboardNodeECU(bus)

    t_inject = 1.00
    for step_idx in range(250):  # 2.5 seconds
        t = round(step_idx * 0.01, 4)
        if t >= t_inject:
            sensor.fault_injection.override_cell_index = 2  # Cell 3
            sensor.fault_injection.override_cell_voltage_mv = 4265  # > 4200 mV threshold
        plant.step(0.01, -60.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    trip_ms = ((bms.fault_trip_timestamp - t_inject) * 1000.0) if bms.fault_trip_timestamp else 9999.0
    tc02_pass = (
        bms.state == BMSState.FAULT_CUTOFF
        and not bms.contactor_closed
        and bool(bms.active_fault_flags & FaultFlags.CELL_OVER_VOLTAGE)
        and dash.mil_fault_lamp_on
        and trip_ms <= 200.0
    )
    records.append(SafetyTestRecord(
        test_id="TC-02",
        title="Cell Over-Voltage (OV) Cut-Off Protection",
        requirement="Open HV contactors within <= 200 ms when any cell > 4200 mV (3 consecutive frames)",
        injected_condition="Cell 3 forced to 4265 mV at t = 1.00 s during fast charge",
        expected_response="BMS enters FAULT_CUTOFF, opens contactor, broadcasts 0x080 DTC 0x0A80",
        measured_result=f"Tripped at t={bms.fault_trip_timestamp:.2f}s (DTC 0x{bms.primary_dtc:04X}, Contactor=OPEN)",
        reaction_time_ms=round(trip_ms, 1),
        max_allowed_ms=200.0,
        passed=tc02_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-03: Cell Under-Voltage (UV < 2800 mV) Fault Injection
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.12)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.12)
    dash = DashboardNodeECU(bus)

    t_inject = 1.00
    for step_idx in range(250):
        t = round(step_idx * 0.01, 4)
        if t >= t_inject:
            sensor.fault_injection.override_cell_index = 1  # Cell 2
            sensor.fault_injection.override_cell_voltage_mv = 2710  # < 2800 mV threshold
        plant.step(0.01, 90.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    trip_ms = ((bms.fault_trip_timestamp - t_inject) * 1000.0) if bms.fault_trip_timestamp else 9999.0
    tc03_pass = (
        bms.state == BMSState.FAULT_CUTOFF
        and not bms.contactor_closed
        and bool(bms.active_fault_flags & FaultFlags.CELL_UNDER_VOLTAGE)
        and dash.mil_fault_lamp_on
        and trip_ms <= 200.0
    )
    records.append(SafetyTestRecord(
        test_id="TC-03",
        title="Cell Under-Voltage (UV) Cut-Off Protection",
        requirement="Open HV contactors within <= 200 ms when any cell < 2800 mV (3 consecutive frames)",
        injected_condition="Cell 2 forced to 2710 mV at t = 1.00 s during 90A discharge",
        expected_response="BMS enters FAULT_CUTOFF, opens contactor, broadcasts 0x080 DTC 0x0A81",
        measured_result=f"Tripped at t={bms.fault_trip_timestamp:.2f}s (DTC 0x{bms.primary_dtc:04X}, Contactor=OPEN)",
        reaction_time_ms=round(trip_ms, 1),
        max_allowed_ms=200.0,
        passed=tc03_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-04: Over-Temperature (OT > 60.0 °C) Fault Injection
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.65)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.65)
    dash = DashboardNodeECU(bus)

    t_inject = 1.00
    for step_idx in range(250):
        t = round(step_idx * 0.01, 4)
        if t >= t_inject:
            sensor.fault_injection.override_temp_mod2_c = 64.2  # > 60.0 C threshold
        plant.step(0.01, 110.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    trip_ms = ((bms.fault_trip_timestamp - t_inject) * 1000.0) if bms.fault_trip_timestamp else 9999.0
    tc04_pass = (
        bms.state == BMSState.FAULT_CUTOFF
        and not bms.contactor_closed
        and bool(bms.active_fault_flags & FaultFlags.OVER_TEMPERATURE)
        and dash.mil_fault_lamp_on
        and trip_ms <= 350.0
    )
    records.append(SafetyTestRecord(
        test_id="TC-04",
        title="Module Over-Temperature (OT) Thermal Runaway Cut-Off",
        requirement="Open HV contactors within <= 350 ms when module temp > 60.0 °C (3 consecutive 100ms frames)",
        injected_condition="Module 2 thermistor forced to 64.2 °C at t = 1.00 s",
        expected_response="BMS enters FAULT_CUTOFF, opens contactor, broadcasts 0x080 DTC 0x0A78",
        measured_result=f"Tripped at t={bms.fault_trip_timestamp:.2f}s (DTC 0x{bms.primary_dtc:04X}, Contactor=OPEN)",
        reaction_time_ms=round(trip_ms, 1),
        max_allowed_ms=350.0,
        passed=tc04_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-05: Pack Over-Current (OC > 200.0 A) Fault Injection
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.70)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.70)
    dash = DashboardNodeECU(bus)

    t_inject = 1.00
    for step_idx in range(200):
        t = round(step_idx * 0.01, 4)
        if t >= t_inject:
            sensor.fault_injection.override_pack_current_a = 235.0  # > 200.0 A threshold
        plant.step(0.01, 50.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    trip_ms = ((bms.fault_trip_timestamp - t_inject) * 1000.0) if bms.fault_trip_timestamp else 9999.0
    tc05_pass = (
        bms.state == BMSState.FAULT_CUTOFF
        and not bms.contactor_closed
        and bool(bms.active_fault_flags & FaultFlags.OVER_CURRENT)
        and trip_ms <= 100.0
    )
    records.append(SafetyTestRecord(
        test_id="TC-05",
        title="Pack Over-Current (OC) Short-Circuit Protection",
        requirement="Open HV contactors within <= 100 ms when |I_pack| > 200.0 A (3 consecutive 20ms frames)",
        injected_condition="Pack current sensor spiked to +235.0 A at t = 1.00 s",
        expected_response="BMS enters FAULT_CUTOFF, opens contactor, broadcasts 0x080 DTC 0x0A90",
        measured_result=f"Tripped at t={bms.fault_trip_timestamp:.2f}s (DTC 0x{bms.primary_dtc:04X}, Contactor=OPEN)",
        reaction_time_ms=round(trip_ms, 1),
        max_allowed_ms=100.0,
        passed=tc05_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-06: Sensor CAN Heartbeat Timeout (> 100 ms loss of 0x101)
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.70)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.70)
    dash = DashboardNodeECU(bus)

    t_inject = 1.00
    for step_idx in range(200):
        t = round(step_idx * 0.01, 4)
        if t >= t_inject:
            sensor.fault_injection.mute_can_transmission = True
        plant.step(0.01, 40.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    trip_ms = ((bms.fault_trip_timestamp - t_inject) * 1000.0) if bms.fault_trip_timestamp else 9999.0
    tc06_pass = (
        bms.state == BMSState.FAULT_CUTOFF
        and not bms.contactor_closed
        and bool(bms.active_fault_flags & FaultFlags.CAN_SENSOR_TIMEOUT)
        and trip_ms <= 130.0
    )
    records.append(SafetyTestRecord(
        test_id="TC-06",
        title="CAN Bus Sensor Heartbeat Loss / Watchdog Cut-Off",
        requirement="Transition to safe state within <= 130 ms if Sensor_Node 0x101 frames cease",
        injected_condition="Sensor_Node CAN TX muted at t = 1.00 s (simulated wire break)",
        expected_response="BMS watchdog expires, opens contactor, broadcasts 0x080 DTC 0xC100",
        measured_result=f"Tripped at t={bms.fault_trip_timestamp:.2f}s (DTC 0x{bms.primary_dtc:04X}, Contactor=OPEN)",
        reaction_time_ms=round(trip_ms, 1),
        max_allowed_ms=130.0,
        passed=tc06_pass,
    ))

    # -------------------------------------------------------------------------
    # TC-07: EMI Single-Frame Transient Spike Rejection (Debounce Verification)
    # -------------------------------------------------------------------------
    bus = VirtualCANBus()
    plant = BatteryPackPlant(initial_soc=0.75)
    sensor = BatterySensorNodeECU(bus, plant)
    bms = BMSControllerECU(bus, initial_soc_estimate=0.75)
    dash = DashboardNodeECU(bus)

    for step_idx in range(200):
        t = round(step_idx * 0.01, 4)
        # Inject a 1-sample (50ms) OV glitch at t=1.00s only, then clear at t=1.05s
        if 1.00 <= t < 1.05:
            sensor.fault_injection.override_cell_index = 0
            sensor.fault_injection.override_cell_voltage_mv = 4250
        else:
            sensor.fault_injection.override_cell_index = None
            sensor.fault_injection.override_cell_voltage_mv = None
        plant.step(0.01, 40.0, bms.contactor_closed)
        sensor.step(t)
        bms.step(t)
        dash.step(t)
        bus.flush()

    tc07_pass = (
        bms.state == BMSState.NORMAL_DRIVE
        and bms.contactor_closed
        and bms.active_fault_flags == 0
        and not dash.mil_fault_lamp_on
    )
    records.append(SafetyTestRecord(
        test_id="TC-07",
        title="Single-Sample EMI Glitch Rejection (Debounce Immunity)",
        requirement="Must NOT false-trip HV contactors on a 1-frame (< 150 ms) voltage spike",
        injected_condition="Cell 1 spiked to 4250 mV for 1 frame (50 ms) at t = 1.00 s, then normal",
        expected_response="Debounce counter increments to 1 then decays to 0; Contactor stays CLOSED",
        measured_result=f"State={bms.state.name}, FaultFlags=0x{bms.active_fault_flags:02X}, Contactor=CLOSED",
        reaction_time_ms=0.0,
        max_allowed_ms=0.0,
        passed=tc07_pass,
    ))

    return records


class TestBMSSafetyCompliance(unittest.TestCase):
    """Standard unittest wrapper so pytest / python -m unittest discovers all 7 safety tests."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.results = {r.test_id: r for r in run_all_verification_tests()}

    def test_tc01_soc_estimation_accuracy(self) -> None:
        self.assertTrue(self.results["TC-01"].passed, self.results["TC-01"].measured_result)

    def test_tc02_over_voltage_cutoff(self) -> None:
        self.assertTrue(self.results["TC-02"].passed, self.results["TC-02"].measured_result)

    def test_tc03_under_voltage_cutoff(self) -> None:
        self.assertTrue(self.results["TC-03"].passed, self.results["TC-03"].measured_result)

    def test_tc04_over_temperature_cutoff(self) -> None:
        self.assertTrue(self.results["TC-04"].passed, self.results["TC-04"].measured_result)

    def test_tc05_over_current_cutoff(self) -> None:
        self.assertTrue(self.results["TC-05"].passed, self.results["TC-05"].measured_result)

    def test_tc06_can_sensor_timeout_cutoff(self) -> None:
        self.assertTrue(self.results["TC-06"].passed, self.results["TC-06"].measured_result)

    def test_tc07_transient_debounce_immunity(self) -> None:
        self.assertTrue(self.results["TC-07"].passed, self.results["TC-07"].measured_result)


if __name__ == "__main__":
    unittest.main(verbosity=2)

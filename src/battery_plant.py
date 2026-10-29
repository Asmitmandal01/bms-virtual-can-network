"""
Electro-Thermal Lithium-Ion Battery Pack Plant Model (4S1P NMC Module).
Models:
- Non-linear Open-Circuit Voltage (OCV) vs. State of Charge (SOC) curve
- 1st-Order Thevenin Equivalent Circuit (Ohmic R0 + RC polarization branch R1/C1)
- Cell-to-cell capacity and impedance variation + passive shunt balancing
- Lumped thermal dynamics (I^2*R Joule heating + convective cooling)
"""

from dataclasses import dataclass, field
from typing import List, Optional
import math


def ocv_from_soc(soc: float) -> float:
    """
    Empirical NMC811 Lithium-Ion Open-Circuit Voltage (OCV) curve in Volts
    for SOC in [0.0, 1.0]. Ranges smoothly from ~2.75V (0% SOC) to ~4.19V (100% SOC).
    """
    s = max(0.001, min(0.999, soc))
    # Combined polynomial + exponential knee model for realistic Li-ion discharge/charge plateau
    v_base = 3.00 + 1.02 * s - 0.38 * (s ** 2) + 0.54 * (s ** 3)
    v_low_knee = -0.25 * math.exp(-18.0 * s)
    v_high_knee = 0.04 * math.exp(-15.0 * (1.0 - s))
    return v_base + v_low_knee + v_high_knee


def soc_from_ocv(ocv_v: float) -> float:
    """Invert the OCV(SOC) curve via binary search (used by BMS for initial/rest OCV lookup)."""
    low, high = 0.0, 1.0
    for _ in range(25):
        mid = 0.5 * (low + high)
        if ocv_from_soc(mid) < ocv_v:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


@dataclass
class BatteryPackPlant:
    """
    Physical simulation of a 4S1P Lithium-Ion EV Battery Module.
    Nominal Capacity: 50.0 Ah, Nominal Pack Voltage: ~14.8V (4 cells in series).
    """
    nominal_capacity_ah: float = 50.0
    initial_soc: float = 0.75
    ambient_temp_c: float = 25.0

    # Per-cell state arrays (4 cells in series)
    cell_soc: List[float] = field(default_factory=list)
    cell_cap_ah: List[float] = field(default_factory=list)
    cell_r0_ohm: List[float] = field(default_factory=list)
    cell_r1_ohm: List[float] = field(default_factory=list)
    cell_c1_farad: List[float] = field(default_factory=list)
    cell_v_rc: List[float] = field(default_factory=list)
    cell_voltages_v: List[float] = field(default_factory=list)

    # Thermal states (Module 1: Cells 1-2, Module 2: Cells 3-4)
    temp_mod1_c: float = 25.0
    temp_mod2_c: float = 25.5
    pack_current_a: float = 0.0  # Convention: positive = Discharge, negative = Charge
    balancing_mask: int = 0      # Bitmask of active passive balancing shunts (bit 0 = Cell 1)

    def __post_init__(self) -> None:
        if not self.cell_soc:
            # Slight manufacturing spread in initial SOC and capacity to demonstrate balancing
            offsets = [0.004, -0.002, 0.001, -0.003]
            self.cell_soc = [max(0.0, min(1.0, self.initial_soc + off)) for off in offsets]
        if not self.cell_cap_ah:
            self.cell_cap_ah = [50.0, 49.8, 50.1, 49.9]
        if not self.cell_r0_ohm:
            self.cell_r0_ohm = [0.0018, 0.0019, 0.0018, 0.0020]
        if not self.cell_r1_ohm:
            self.cell_r1_ohm = [0.0012, 0.0012, 0.0013, 0.0012]
        if not self.cell_c1_farad:
            self.cell_c1_farad = [1800.0, 1800.0, 1750.0, 1820.0]
        if not self.cell_v_rc:
            self.cell_v_rc = [0.0, 0.0, 0.0, 0.0]
        if not self.cell_voltages_v:
            self.cell_voltages_v = [ocv_from_soc(s) for s in self.cell_soc]

    @property
    def true_pack_soc(self) -> float:
        return sum(self.cell_soc) / len(self.cell_soc)

    @property
    def pack_voltage_v(self) -> float:
        return sum(self.cell_voltages_v)

    def step(self, dt_s: float, commanded_current_a: float, contactor_closed: bool) -> None:
        """
        Advance the electro-thermal battery model by dt_s seconds.
        If HV contactors are open, actual pack current is forced to 0.0 A.
        """
        self.pack_current_a = commanded_current_a if contactor_closed else 0.0

        total_joule_heat_w = 0.0
        for i in range(4):
            # Passive bleed current (shunt resistor bleed current)
            bleed_a = 0.85 if (self.balancing_mask & (1 << i)) else 0.0
            cell_i_a = self.pack_current_a + bleed_a

            # Update SOC (Coulomb integration: positive current discharges cell)
            dsoc = -(cell_i_a * dt_s) / (self.cell_cap_ah[i] * 3600.0)
            self.cell_soc[i] = max(0.0, min(1.0, self.cell_soc[i] + dsoc))

            # Update RC polarization voltage
            tau = self.cell_r1_ohm[i] * self.cell_c1_farad[i]
            alpha = math.exp(-dt_s / tau)
            self.cell_v_rc[i] = self.cell_v_rc[i] * alpha + self.cell_r1_ohm[i] * cell_i_a * (1.0 - alpha)

            # Terminal voltage: V_term = OCV(SOC) - I * R0 - V_RC
            ocv = ocv_from_soc(self.cell_soc[i])
            v_term = ocv - cell_i_a * self.cell_r0_ohm[i] - self.cell_v_rc[i]
            self.cell_voltages_v[i] = max(2.0, min(4.6, v_term))

            # Heating contribution (amplified effective pack resistance for realistic thermal rise)
            r_eff = (self.cell_r0_ohm[i] + self.cell_r1_ohm[i]) * 4.5
            total_joule_heat_w += (cell_i_a ** 2) * r_eff

        # Lumped thermal dynamics: C_th * dT/dt = Q_joule - h * (T - T_amb)
        c_th = 2400.0  # Thermal mass J/degC (50Ah 4S module with cold-plate)
        h_cool = 22.0  # Active liquid/forced-air cooling coefficient W/degC
        q_mod1 = 0.48 * total_joule_heat_w
        q_mod2 = 0.52 * total_joule_heat_w

        self.temp_mod1_c += ((q_mod1 - h_cool * (self.temp_mod1_c - self.ambient_temp_c)) / c_th) * dt_s
        self.temp_mod2_c += ((q_mod2 - h_cool * (self.temp_mod2_c - self.ambient_temp_c)) / c_th) * dt_s

import numpy as np
import openmdao.api as om
from scipy.constants import pi

from .sizing_scenarios import HoverAndTO
from .propeller import Propeller
from .motor import Motor
from .battery_and_esc import BatteryAndESC
from .frame import Frame
from .mission import Mission
from .trajectory import Model


class MTOW(om.ExplicitComponent):
    """
    Computes the separation angle between rotors
    """

    def setup(self):
        self.add_input("data:system:payload", val=np.nan, units="kg")
        self.add_input("data:frame:mass", val=np.nan, units="kg")
        self.add_input("data:battery:mass", val=np.nan, units="kg")
        self.add_input("data:esc:mass", val=np.nan, units="kg")
        self.add_input("data:motor:mass", val=np.nan, units="kg")
        self.add_input("data:propeller:mass", val=np.nan, units="kg")
        self.add_input("data:propeller:number", val=np.nan, units=None)

        self.add_output("data:system:MTOW", units="kg")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        M_pay = inputs["data:system:payload"]
        M_frame = inputs["data:frame:mass"]
        M_bat = inputs["data:battery:mass"]
        M_esc = inputs["data:esc:mass"]
        M_pro = inputs["data:propeller:mass"]
        M_mot = inputs["data:motor:mass"]
        N_pro = inputs["data:propeller:number"]

        M_total = (M_esc + M_pro + M_mot) * N_pro + M_pay + M_bat + M_frame

        outputs["data:system:MTOW"] = M_total


class ConstraintMTOW(om.ExplicitComponent):
    """
    Computes the constraint to converge MTOW
    """

    def setup(self):
        self.add_input("data:system:MTOW", val=np.nan, units="kg")
        self.add_input("data:system:MTOW:guess", val=np.nan, units="kg")

        self.add_output("data:system:MTOW:constraint", units="kg")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        MTOW = inputs["data:system:MTOW"]
        MTOW_est = inputs["data:system:MTOW:guess"]

        MTOW_constr = MTOW - MTOW_est

        outputs["data:system:MTOW:constraint"] = MTOW_constr

    def compute_partials(self, inputs, partials, discrete_inputs=None):
        partials["data:system:MTOW:constraint", "data:system:MTOW"] = 1.0
        partials["data:system:MTOW:constraint", "data:system:MTOW:guess"] = -1.0


class ConstraintBatteryVoltage(om.ExplicitComponent):
    """
    Computes the constraint for battery voltage
    """

    def setup(self):
        self.add_input("data:battery:voltage", val=np.nan, units="V")
        self.add_input("data:motor:voltage:takeoff", val=np.nan, units="V")

        self.add_output("data:battery:voltage:constraint", units="V")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        U_bat = inputs["data:battery:voltage"]
        U_mot = inputs["data:motor:voltage:takeoff"]

        U_bat_constr = U_mot - U_bat

        outputs["data:battery:voltage:constraint"] = U_bat_constr

    def compute_partials(self, inputs, partials, discrete_inputs=None):
        partials["data:battery:voltage:constraint", "data:battery:voltage"] = -1.0
        partials["data:battery:voltage:constraint", "data:motor:voltage:takeoff"] = 1.0


class ConstraintESCVoltage(om.ExplicitComponent):
    """
    Computes the constraint for esc voltage
    """

    def setup(self):
        self.add_input("data:esc:voltage", val=np.nan, units="V")
        self.add_input("data:motor:voltage:takeoff", val=np.nan, units="V")

        self.add_output("data:esc:voltage:constraint", units="V")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        U_esc = inputs["data:esc:voltage"]
        U_mot = inputs["data:motor:voltage:takeoff"]

        U_esc_constr = U_mot - U_esc

        outputs["data:esc:voltage:constraint"] = U_esc_constr

    def compute_partials(self, inputs, partials, discrete_inputs=None):
        partials["data:esc:voltage:constraint", "data:esc:voltage"] = -1.0
        partials["data:esc:voltage:constraint", "data:motor:voltage:takeoff"] = 1.0


class ConstraintBatteryEnergy(om.ExplicitComponent):
    """
    Computes the constraint for the battery energy
    """

    def setup(self):
        self.add_input("data:battery:energy", val=np.nan, units="J")
        self.add_input("data:mission:energy", val=np.nan, units="J")

        self.add_output("data:battery:energy:constraint", units="J")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        NRJ_bat = inputs["data:battery:energy"]
        NRJ_mission = inputs["data:mission:energy"]

        NRJ_bat_constr = NRJ_mission/NRJ_bat - 1

        outputs["data:battery:energy:constraint"] = NRJ_bat_constr

    def compute_partials(self, inputs, partials, discrete_inputs=None):
        NRJ_bat = inputs["data:battery:energy"]
        NRJ_mission = inputs["data:mission:energy"]
        partials["data:battery:energy:constraint", "data:battery:energy"] = -NRJ_mission/NRJ_bat**2
        partials["data:battery:energy:constraint", "data:mission:energy"] = 1.0/NRJ_bat


class SystemConstraints(om.Group):
    def setup(self):
        self.add_subsystem("mtow", MTOW(), promotes=["*"])
        self.add_subsystem("constraint_mtow", ConstraintMTOW(), promotes=["*"])
        self.add_subsystem(
            "constraint_battery_voltage", ConstraintBatteryVoltage(), promotes=["*"]
        )
        self.add_subsystem(
            "constraint_esc_voltage", ConstraintESCVoltage(), promotes=["*"]
        )
        self.add_subsystem(
            "constraint_battery_energy", ConstraintBatteryEnergy(), promotes=["*"]
        )


class Multirotor(om.Group):
    def setup(self):
        self.add_subsystem("sizing_scenarios", HoverAndTO(), promotes=["*"])
        self.add_subsystem("propeller", Propeller(), promotes=["*"])
        self.add_subsystem("motor", Motor(), promotes=["*"])
        self.add_subsystem("trajectory", Model(), promotes=["*"])
        self.add_subsystem("battery_and_esc", BatteryAndESC(), promotes=["*"])
        self.add_subsystem("frame", Frame(), promotes=["*"])
        self.add_subsystem("mission", Mission(), promotes=["*"])
        self.add_subsystem("mtow", MTOW(), promotes=["*"])
        self.add_subsystem("system_constraints", SystemConstraints(), promotes=["*"])

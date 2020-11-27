import numpy as np
import openmdao.api as om


class BatteryVoltage(om.ExplicitComponent):
    """
    Computes battery voltage
    """

    def setup(self):
        self.add_input("data:battery:cell:voltage", val=np.nan, units="V")
        self.add_input("data:battery:voltage:guess", val=np.nan, units="V")

        self.add_output("data:battery:voltage", units="V")
        self.add_output("data:battery:cell:number", units=None)

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        U_cell = inputs["data:battery:cell:voltage"]
        U_bat_est = inputs["data:battery:voltage:guess"]

        N_s_bat = np.ceil(U_bat_est / U_cell)
        U_bat = 3.7 * N_s_bat

        outputs["data:battery:voltage"] = U_bat
        outputs["data:battery:cell:number"] = N_s_bat


class BatteryMass(om.ExplicitComponent):
    """
    Computes battery mass and capacity
    """

    def setup(self):
        self.add_input("data:battery:mass:k", val=np.nan, units=None)
        self.add_input("data:system:payload", val=np.nan, units="kg")

        self.add_output("data:battery:mass", units="kg")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        k_os = inputs["data:battery:mass:k"]
        payload = inputs["data:system:payload"]

        M = k_os * payload

        outputs["data:battery:mass"] = M


class BatteryEnergy(om.ExplicitComponent):
    """
    Computes nominal torque for hover
    """

    def setup(self):
        self.add_input("data:battery:energy:ref", val=np.nan, units="J")
        self.add_input("data:battery:mass", val=np.nan, units="kg")
        self.add_input("data:battery:mass:ref", val=np.nan, units="kg")
        self.add_input("data:battery:capacity:discharge_limit", val=np.nan, units=None)
        self.add_input("data:battery:voltage", val=np.nan, units="V")

        self.add_output("data:battery:energy", units="J")
        self.add_output("data:battery:capacity", units="A*s")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        E_bat_ref = inputs["data:battery:energy:ref"]
        M = inputs["data:battery:mass"]
        M_ref = inputs["data:battery:mass:ref"]
        C_ratio = inputs["data:battery:capacity:discharge_limit"]
        U_bat = inputs["data:battery:voltage"]

        E_bat = E_bat_ref * M / M_ref * (1 - C_ratio)
        C_bat = E_bat / U_bat

        outputs["data:battery:energy"] = E_bat
        outputs["data:battery:capacity"] = C_bat


class ESCPower(om.ExplicitComponent):
    """
    Computes max ESC power
    """

    def setup(self):
        self.add_input("data:motor:power:takeoff", val=np.nan, units="W")
        self.add_input("data:motor:voltage:takeoff", val=np.nan, units="V")
        self.add_input("data:battery:voltage", val=np.nan, units="V")

        self.add_output("data:esc:power", units="W")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        P_mot_to = inputs["data:motor:power:takeoff"]
        U_mot_to = inputs["data:motor:voltage:takeoff"]
        U_bat = inputs["data:battery:voltage"]

        P_esc = P_mot_to * U_bat / U_mot_to

        outputs["data:esc:power"] = P_esc


class ESCMassAndVoltage(om.ExplicitComponent):
    """
    Computes ESC mass and voltage
    """

    def setup(self):
        self.add_input("data:esc:power", val=np.nan, units="W")
        self.add_input("data:esc:power:ref", val=np.nan, units="W")
        self.add_input("data:esc:mass:ref", val=np.nan, units="kg")

        self.add_output("data:esc:mass", units="kg")
        self.add_output("data:esc:voltage", units="V")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        P_esc = inputs["data:esc:power"]
        P_esc_ref = inputs["data:esc:power:ref"]
        M_esc_ref = inputs["data:esc:mass:ref"]

        M_esc = M_esc_ref * (P_esc / P_esc_ref)
        V_esc = 1.84 * P_esc ** 0.36

        outputs["data:esc:mass"] = M_esc
        outputs["data:esc:voltage"] = V_esc


class BatteryAndESC(om.Group):
    def setup(self):
        self.add_subsystem("battery_voltage", BatteryVoltage(), promotes=["*"])
        self.add_subsystem("battery_mass", BatteryMass(), promotes=["*"])
        self.add_subsystem("battery_energy", BatteryEnergy(), promotes=["*"])
        self.add_subsystem("esc_power", ESCPower(), promotes=["*"])
        self.add_subsystem("esc_mass_voltage", ESCMassAndVoltage(), promotes=["*"])

import numpy as np
from scipy.constants import pi
import openmdao.api as om
from fastoad.utils.physics import AtmosphereSI


class PowerAndThrustCoefficients(om.ExplicitComponent):
    """
    Computes power and thrust coefficients
    """

    def setup(self):
        self.add_input("data:propeller:pitch", val=np.nan, units=None)

        self.add_output("data:propeller:Ct", units=None)
        self.add_output("data:propeller:Cp", units=None)

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        pitch = inputs["data:propeller:pitch"]

        Ct = 4.27e-02 + 1.44e-01 * pitch
        Cp = -1.48e-03 + 9.72e-02 * pitch

        outputs["data:propeller:Ct"] = Ct
        outputs["data:propeller:Cp"] = Cp


class Diameter(om.ExplicitComponent):
    """
    Computes diameter based on takeoff scenario
    """

    def setup(self):
        self.add_input("data:propeller:force:takeoff:z", val=np.nan, units="N")
        self.add_input("data:propeller:Ct", val=np.nan, units=None)
        self.add_input("settings:altitude", val=np.nan, units="m")
        self.add_input("data:propeller:ND:k", val=np.nan, units=None)
        self.add_input("data:propeller:ND:max", val=np.nan, units=None)

        self.add_output("data:propeller:ND", units=None)
        self.add_output("data:propeller:diameter", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        F_pro = inputs["data:propeller:force:takeoff:z"]
        Ct = inputs["data:propeller:Ct"]
        altitude = inputs["settings:altitude"]
        k_ND = inputs["data:propeller:ND:k"]
        ND_max = inputs["data:propeller:ND:max"]

        atm = AtmosphereSI(altitude)
        rho = atm.density

        ND = ND_max / k_ND
        D_pro = (F_pro / (Ct * rho * ND ** 2.0)) ** 0.5

        outputs["data:propeller:ND"] = ND
        outputs["data:propeller:diameter"] = D_pro


class RotationalSpeedTakeoff(om.ExplicitComponent):
    """
    Computes speed based on takeoff scenario
    """

    def setup(self):
        self.add_input("data:propeller:ND", val=np.nan, units=None)
        self.add_input("data:propeller:diameter", val=np.nan, units="")

        self.add_output("data:propeller:frequency:takeoff", units="Hz")
        self.add_output("data:propeller:speed:takeoff", units="rad/s")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        ND = inputs["data:propeller:ND"]
        D = inputs["data:propeller:diameter"]

        n = ND / D
        omega = n * 2 * pi

        outputs["data:propeller:frequency:takeoff"] = n
        outputs["data:propeller:speed:takeoff"] = omega


class MassAndInertia(om.ExplicitComponent):
    """
    Computes mass and inertia based on takeoff scenario
    """

    def setup(self):
        self.add_input("data:propeller:mass:ref", val=np.nan, units="kg")
        self.add_input("data:propeller:diameter:ref", val=np.nan, units="m")
        self.add_input("data:propeller:diameter", val=np.nan, units="m")

        self.add_output("data:propeller:mass", units="kg")
        self.add_output("data:propeller:inertia", units="kg*m**2")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        M_ref = inputs["data:propeller:mass:ref"]
        D_ref = inputs["data:propeller:diameter:ref"]
        D = inputs["data:propeller:diameter"]

        M = M_ref * (D / D_ref) ** 2.0
        J = M * (D / 2) ** 2 / 3

        outputs["data:propeller:mass"] = M
        outputs["data:propeller:inertia"] = J


class PowerAndTorqueTakeoff(om.ExplicitComponent):
    """
    Computes power and torque based on takeoff scenario
    """

    def setup(self):
        self.add_input("data:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:propeller:frequency:takeoff", val=np.nan, units="Hz")
        self.add_input("data:propeller:speed:takeoff", val=np.nan, units="rad/s")
        self.add_input("data:propeller:Cp", val=np.nan, units=None)
        self.add_input("settings:altitude", val=np.nan, units="m")

        self.add_output("data:propeller:power:takeoff", units="W")
        self.add_output("data:propeller:torque:takeoff", units="N*m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        D = inputs["data:propeller:diameter"]
        n = inputs["data:propeller:frequency:takeoff"]
        omega = inputs["data:propeller:speed:takeoff"]
        Cp = inputs["data:propeller:Cp"]
        altitude = inputs["settings:altitude"]

        atm = AtmosphereSI(altitude)
        rho = atm.density

        P = Cp * rho * n ** 3.0 * D ** 5.0
        T = P / omega

        outputs["data:propeller:power:takeoff"] = P
        outputs["data:propeller:torque:takeoff"] = T


class RotationalSpeedHover(om.ExplicitComponent):
    """
    Computes speed based on hover scenario
    """

    def setup(self):
        self.add_input("data:propeller:force:hover:z", val=np.nan, units="N")
        self.add_input("data:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:propeller:Ct", val=np.nan, units=None)
        self.add_input("settings:altitude", val=np.nan, units="m")

        self.add_output("data:propeller:frequency:hover", units="Hz")
        self.add_output("data:propeller:speed:hover", units="rad/s")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        F = inputs["data:propeller:force:hover:z"]
        D = inputs["data:propeller:diameter"]
        Ct = inputs["data:propeller:Ct"]
        altitude = inputs["settings:altitude"]

        atm = AtmosphereSI(altitude)
        rho = atm.density

        n = np.sqrt(F / (Ct * rho * D ** 4.0))
        omega = n * 2.0 * pi

        outputs["data:propeller:frequency:hover"] = n
        outputs["data:propeller:speed:hover"] = omega


class PowerAndTorqueHover(om.ExplicitComponent):
    """
    Computes power and torque based on hover scenario
    """

    def setup(self):
        self.add_input("data:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:propeller:frequency:hover", val=np.nan, units="Hz")
        self.add_input("data:propeller:speed:hover", val=np.nan, units="rad/s")
        self.add_input("data:propeller:Cp", val=np.nan, units=None)
        self.add_input("settings:altitude", val=np.nan, units="m")

        self.add_output("data:propeller:power:hover", units="W")
        self.add_output("data:propeller:torque:hover", units="N*m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        D = inputs["data:propeller:diameter"]
        n = inputs["data:propeller:frequency:hover"]
        omega = inputs["data:propeller:speed:hover"]
        Cp = inputs["data:propeller:Cp"]
        altitude = inputs["settings:altitude"]

        atm = AtmosphereSI(altitude)
        rho = atm.density

        P = Cp * rho * n ** 3.0 * D ** 5.0
        T = P / omega

        outputs["data:propeller:power:hover"] = P
        outputs["data:propeller:torque:hover"] = T


class Propeller(om.Group):
    def setup(self):
        self.add_subsystem(
            "power_thrust_coefficients", PowerAndThrustCoefficients(), promotes=["*"]
        )
        self.add_subsystem("diameter", Diameter(), promotes=["*"])
        self.add_subsystem("speed_takeoff", RotationalSpeedTakeoff(), promotes=["*"])
        self.add_subsystem("mass_inertia", MassAndInertia(), promotes=["*"])
        self.add_subsystem(
            "power_torque_takeoff", PowerAndTorqueTakeoff(), promotes=["*"]
        )
        self.add_subsystem("speed_hover", RotationalSpeedHover(), promotes=["*"])
        self.add_subsystem("power_torque_hover", PowerAndTorqueHover(), promotes=["*"])

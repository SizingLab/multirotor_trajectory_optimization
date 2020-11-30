import numpy as np
import openmdao.api as om
from scipy.constants import pi


class SeparationAngle(om.ExplicitComponent):
    """
    Computes the separation angle between rotors
    """

    def setup(self):
        self.add_input("data:frame:arm:number", val=np.nan, units=None)

        self.add_output("data:frame:separation_angle", units="rad")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        N_arm = inputs["data:frame:arm:number"]

        alpha_sep = 2 * pi / N_arm

        outputs["data:frame:separation_angle"] = alpha_sep


class ArmLength(om.ExplicitComponent):
    """
    Computes the arm length
    """

    def setup(self):
        self.add_input("data:frame:separation_angle", val=np.nan, units="rad")
        self.add_input("data:propeller:diameter", val=np.nan, units="m")

        self.add_output("data:frame:arm:length", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        alpha_sep = inputs["data:frame:separation_angle"]
        D_pro = inputs["data:propeller:diameter"]

        L_arm = D_pro / (2.0 * np.sin(alpha_sep / 2.0))

        outputs["data:frame:arm:length"] = L_arm


class TubeDiameterAndThickness(om.ExplicitComponent):
    """
    Computes the arm (tube) diameter and thickness
    """

    def setup(self):
        self.add_input("data:propeller:force:takeoff:z", val=np.nan, units="N")
        self.add_input("data:frame:arm:propeller_number", val=np.nan, units=None)
        self.add_input("data:frame:arm:stress:max", val=np.nan, units="Pa")
        self.add_input("data:frame:arm:length", val=np.nan, units="m")
        self.add_input("data:frame:arm:diameter:k", val=np.nan, units=None)

        self.add_output("data:frame:arm:diameter:inner", units="m")
        self.add_output("data:frame:arm:diameter:outer", units="m")
        self.add_output("data:frame:arm:thickness", units="m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        F_pro_to = inputs["data:propeller:force:takeoff:z"]
        N_pro_arm = inputs["data:frame:arm:propeller_number"]
        sigma_max = inputs["data:frame:arm:stress:max"]
        L_arm = inputs["data:frame:arm:length"]
        k_D = inputs["data:frame:arm:diameter:k"]

        D_out_arm = (
            F_pro_to
            * N_pro_arm
            / sigma_max
            * L_arm
            * 32.0
            / (pi * (1.0 - (1.0 - 2.0 * k_D) ** 4.0))
        ) ** (1 / 3)
        e_arm = k_D * D_out_arm
        D_in_arm = D_out_arm - 2 * e_arm

        outputs["data:frame:arm:diameter:inner"] = D_in_arm
        outputs["data:frame:arm:diameter:outer"] = D_out_arm
        outputs["data:frame:arm:thickness"] = e_arm


class Mass(om.ExplicitComponent):
    """
    Computes the arm and frame mass
    """

    def setup(self):
        self.add_input("data:frame:arm:diameter:outer", val=np.nan, units="m")
        self.add_input("data:frame:arm:thickness", val=np.nan, units="m")
        self.add_input("data:frame:arm:density", val=np.nan, units="kg/m**3")
        self.add_input("data:frame:arm:length", val=np.nan, units="m")
        self.add_input("data:frame:arm:number", val=np.nan, units=None)

        self.add_output("data:frame:arm:mass", units="kg")
        self.add_output("data:frame:mass", units="kg")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        D_out_arm = inputs["data:frame:arm:diameter:outer"]
        e_arm = inputs["data:frame:arm:thickness"]
        rho_s = inputs["data:frame:arm:density"]
        L_arm = inputs["data:frame:arm:length"]
        N_arm = inputs["data:frame:arm:number"]

        M_arm = (
            pi
            / 4.0
            * (D_out_arm ** 2.0 - (D_out_arm - 2.0 * e_arm) ** 2)
            * L_arm
            * rho_s
        )
        M_frame = N_arm * M_arm / 0.4

        outputs["data:frame:arm:mass"] = M_arm
        outputs["data:frame:mass"] = M_frame


class Frame(om.Group):
    def setup(self):
        self.add_subsystem("separation_angle", SeparationAngle(), promotes=["*"])
        self.add_subsystem("arm_length", ArmLength(), promotes=["*"])
        self.add_subsystem(
            "diameter_and_thickness", TubeDiameterAndThickness(), promotes=["*"]
        )
        self.add_subsystem("mass", Mass(), promotes=["*"])

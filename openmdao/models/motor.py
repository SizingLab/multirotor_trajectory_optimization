import numpy as np
import openmdao.api as om


class NominalTorque(om.ExplicitComponent):
    """
    Computes nominal torque for hover
    """

    def setup(self):
        self.add_input("data:propeller:torque:hover", val=np.nan, units="N*m")
        self.add_input("data:motor:torque:hover:k", val=np.nan, units=None)

        self.add_output("data:motor:torque:hover", units="N*m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        T_pro = inputs["data:propeller:torque:hover"]
        k_os = inputs["data:motor:torque:hover:k"]

        T_mot = k_os * T_pro

        outputs["data:motor:torque:hover"] = T_mot


class TorqueConstant(om.ExplicitComponent):
    """
    Computes torque constant
    """

    def setup(self):
        self.add_input("data:motor:speed:k", val=np.nan, units=None)
        self.add_input("data:propeller:speed:takeoff", val=np.nan, units="rad/s")
        self.add_input("data:battery:voltage:guess", val=np.nan, units="V")

        self.add_output("data:motor:Kt", units="N*m/A")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        k_os = inputs["data:motor:speed:k"]
        omega_pro = inputs["data:propeller:speed:takeoff"]
        U_bat = inputs["data:battery:voltage:guess"]

        K_mot = U_bat / (k_os * omega_pro)

        outputs["data:motor:Kt"] = K_mot


class ScalingLaws(om.ExplicitComponent):
    """
    Computes the different scaling laws
    """

    def setup(self):
        self.add_input("data:motor:torque:hover", val=np.nan, units="N*m")
        self.add_input("data:motor:torque:nominal:ref", val=np.nan, units="N*m")
        self.add_input("data:motor:torque:max:ref", val=np.nan, units="N*m")
        self.add_input("data:motor:Kt", val=np.nan, units="N*m/A")
        self.add_input("data:motor:Kt:ref", val=np.nan, units="N*m/A")
        self.add_input("data:motor:mass:ref", val=np.nan, units="kg")
        self.add_input("data:motor:resistance:ref", val=np.nan, units="Ohm")
        self.add_input("data:motor:friction:ref", val=np.nan, units="N*m/A")

        self.add_output("data:motor:mass", units="kg")
        self.add_output("data:motor:resistance", units="Ohm")
        self.add_output("data:motor:friction", units="N*m")
        self.add_output("data:motor:torque:max", units="N*m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        T_nom = inputs["data:motor:torque:hover"]
        T_nom_ref = inputs["data:motor:torque:nominal:ref"]
        T_max_ref = inputs["data:motor:torque:max:ref"]
        Kt = inputs["data:motor:Kt"]
        Kt_ref = inputs["data:motor:Kt:ref"]
        M_ref = inputs["data:motor:mass:ref"]
        R_ref = inputs["data:motor:resistance:ref"]
        T_fr_ref = inputs["data:motor:friction:ref"]

        M_mot = M_ref * (T_nom / T_nom_ref) ** (3.0 / 3.5)
        R_mot = R_ref * (T_nom / T_nom_ref) ** (-5.0 / 3.5) * (Kt / Kt_ref) ** 2.0
        T_mot_fr = T_fr_ref * (T_nom / T_nom_ref) ** (3.0 / 3.5)
        T_max_mot = T_max_ref * (T_nom / T_nom_ref)

        outputs["data:motor:mass"] = M_mot
        outputs["data:motor:resistance"] = R_mot
        outputs["data:motor:friction"] = T_mot_fr
        outputs["data:motor:torque:max"] = T_max_mot


class CurrentAndVoltage(om.ExplicitComponent):
    """
    Computes the current, voltage and power
    """

    def setup(self):
        self.add_input("data:propeller:torque:hover", val=np.nan, units="N*m")
        self.add_input("data:propeller:speed:hover", val=np.nan, units="rad/s")
        self.add_input("data:propeller:torque:takeoff", val=np.nan, units="N*m")
        self.add_input("data:propeller:speed:takeoff", val=np.nan, units="rad/s")
        self.add_input("data:motor:Kt", val=np.nan, units="N*m/A")
        self.add_input("data:motor:friction", val=np.nan, units="N*m")
        self.add_input("data:motor:resistance", val=np.nan, units="Ohm")

        self.add_output("data:motor:current:hover", units="A")
        self.add_output("data:motor:current:takeoff", units="A")
        self.add_output("data:motor:voltage:hover", units="V")
        self.add_output("data:motor:voltage:takeoff", units="V")
        self.add_output("data:motor:power:hover", units="W")
        self.add_output("data:motor:power:takeoff", units="W")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        T_pro_hov = inputs["data:propeller:torque:hover"]
        omega_speed_hov = inputs["data:propeller:speed:hover"]
        T_pro_to = inputs["data:propeller:torque:takeoff"]
        omega_speed_to = inputs["data:propeller:speed:takeoff"]
        Kt = inputs["data:motor:Kt"]
        T_mot_fr = inputs["data:motor:friction"]
        R_mot = inputs["data:motor:resistance"]

        # Hover current and voltage
        I_mot_hov = (T_pro_hov + T_mot_fr) / Kt
        U_mot_hov = R_mot * I_mot_hov + omega_speed_hov * Kt
        P_el_mot_hov = U_mot_hov * I_mot_hov

        # Takeoff current and voltage
        I_mot_to = (T_pro_to + T_mot_fr) / Kt
        U_mot_to = R_mot * I_mot_to + omega_speed_to * Kt
        P_el_mot_to = U_mot_to * I_mot_to

        outputs["data:motor:current:hover"] = I_mot_hov
        outputs["data:motor:current:takeoff"] = I_mot_to
        outputs["data:motor:voltage:hover"] = U_mot_hov
        outputs["data:motor:voltage:takeoff"] = U_mot_to
        outputs["data:motor:power:hover"] = P_el_mot_hov
        outputs["data:motor:power:takeoff"] = P_el_mot_to


class ConstraintMotorTorque(om.ExplicitComponent):
    """
    Computes the constraint for the motor torque
    """

    def setup(self):
        self.add_input("data:propeller:torque:takeoff", val=np.nan, units="N*m")
        self.add_input("data:motor:torque:max", val=np.nan, units="N*m")

        self.add_output("data:motor:torque:max:constraint", units="N*m")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        T_pro = inputs["data:propeller:torque:takeoff"]
        T_mot = inputs["data:motor:torque:max"]

        T_mot_constr = T_pro - T_mot

        outputs["data:motor:torque:max:constraint"] = T_mot_constr

    def compute_partials(self, inputs, partials, discrete_inputs=None):
        partials["data:motor:torque:max:constraint", "data:motor:torque:max"] = -1.
        partials["data:motor:torque:max:constraint", "data:propeller:torque:takeoff"] = 1.


class Motor(om.Group):
    def setup(self):
        self.add_subsystem("nominal_torque", NominalTorque(), promotes=["*"])
        self.add_subsystem("torque_constant", TorqueConstant(), promotes=["*"])
        self.add_subsystem("scaling_laws", ScalingLaws(), promotes=["*"])
        self.add_subsystem("current_and_voltage", CurrentAndVoltage(), promotes=["*"])
        self.add_subsystem("torque_constraint", ConstraintMotorTorque(), promotes=["*"])

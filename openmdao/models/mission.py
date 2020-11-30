import numpy as np
import openmdao.api as om
from scipy.constants import g


class Mission(om.ExplicitComponent):
    """
    Computes the energy for the mission
    """
    def initialize(self):
        # To perform an evaluation instead of a design
        self.options.declare("evaluation", types=bool, default=False)

        # To use potential an approximation of the trajectory analysis
        self.options.declare("use_approximation", types=bool, default=False)

    def setup(self):

        self.add_input("data:propeller:number", val=np.nan, units=None)
        self.add_input("data:esc:efficiency", val=np.nan, units=None)
        self.add_input("data:motor:power:hover", val=np.nan, units="W")
        self.add_input("data:motor:power:takeoff", val=np.nan, units="W")
        self.add_input("data:mission:hover:time", val=np.nan, units="s")
        self.add_input("data:mission:travel:time", val=2.8, units="s")
        self.add_input("data:mission:travel:displacement:number", val=np.nan, units=None)


        # self.add_input("data:mission:travel:displacement:height", val=10.0, units="m")
        # self.add_input("data:system:MTOW:guess", val=np.nan, units="kg")

        self.add_input("data:mission:travel:energy", val=np.nan, units="J")

        if self.options["evaluation"]:
            self.add_input("data:mission:energy", val=np.nan, units="J")

        self.add_output("data:mission:travel:displacement:number:guess", units=None)
        self.add_output("data:mission:energy:guess", units="J")
        self.add_output("data:mission:hover:energy", units="J")
        self.add_output("data:mission:hover:time:max", units="s")

        if not self.options["evaluation"]:
            self.add_output("data:mission:energy", units="J")


        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):

        N_pro = inputs["data:propeller:number"]
        ESC_eta = inputs["data:esc:efficiency"]
        P_el_mot_hov = inputs["data:motor:power:hover"]
        P_el_mot_takeoff = inputs["data:motor:power:takeoff"]
        t_hov = inputs["data:mission:hover:time"]
        t_travel = inputs["data:mission:travel:time"]
        NRJ_Elevation = inputs["data:mission:travel:energy"]
        N_dis = inputs["data:mission:travel:displacement:number"]

        E_hover = (P_el_mot_hov * N_pro) / ESC_eta * t_hov

        # MTOW = inputs["data:system:MTOW:guess"]
        # H_dis = inputs["data:mission:travel:displacement:height"]
        if self.options["use_approximation"]:
            # NRJ_Elevation_est = (MTOW * g * H_dis) * 2 / 0.8 / 0.8 / ESC_eta
            NRJ_Elevation_est = (P_el_mot_takeoff * N_pro) / ESC_eta * t_travel
        else:
            NRJ_Elevation_est = NRJ_Elevation
        E_drone_est = (NRJ_Elevation_est + E_hover) * N_dis

        if self.options["evaluation"]:
            E_drone = inputs["data:mission:energy"]
        else:
            E_drone = (E_hover + NRJ_Elevation) * N_dis

        N_dis_guess = np.floor(E_drone/E_drone_est*N_dis)

        if self.options["use_approximation"]:
            E_drone = E_drone_est

        t_hov_max = E_drone/(P_el_mot_hov * N_pro) / ESC_eta

        outputs["data:mission:travel:displacement:number:guess"] = N_dis_guess
        outputs["data:mission:energy:guess"] = E_drone_est
        outputs["data:mission:hover:energy"] = E_hover
        outputs["data:mission:hover:time:max"] = t_hov_max
        if not self.options["evaluation"]:
            outputs["data:mission:energy"] = E_drone

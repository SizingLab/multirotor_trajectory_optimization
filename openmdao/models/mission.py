import numpy as np
import openmdao.api as om


class Mission(om.ExplicitComponent):
    """
    Computes the energy for the mission
    """

    def setup(self):
        # self.add_input("data:mission:traval:displacement:height", val=np.nan, units="m")
        self.add_input(
            "data:mission:travel:displacement:number", val=np.nan, units=None
        )
        # self.add_input("data:mission:travel:time", val=np.nan, units="s")
        self.add_input("data:mission:hover:time", val=np.nan, units="s")
        self.add_input("data:mission:travel:energy", val=np.nan, units="J")
        self.add_input("data:propeller:number", val=np.nan, units=None)
        self.add_input("data:esc:efficiency", val=np.nan, units=None)
        self.add_input("data:motor:power", val=np.nan, units=None)

        self.add_output("data:mission:hover:energy", units="J")
        self.add_output("data:mission:energy", units="J")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        N_dis = inputs["data:mission:travel:displacement:number"]
        t_hov = inputs["data:mission:hover:time"]
        NRJ_Elevation = inputs["data:mission:travel:energy"]
        N_pro = inputs["data:propeller:number"]
        ESC_eta = inputs["data:esc:efficiency"]
        P_el_mot_hov = inputs["data:motor:power"]

        E_hover = (P_el_mot_hov * N_pro) / ESC_eta * t_hov * 60
        E_drone = E_hover + NRJ_Elevation * N_dis

        outputs["data:battery:voltage:guess"] = E_hover
        outputs["data:battery:cell:number"] = E_drone

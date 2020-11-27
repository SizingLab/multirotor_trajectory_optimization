import numpy as np
import openmdao.api as om
from scipy.constants import g


class HoverAndTO(om.ExplicitComponent):
    """
    Computes hover and takeoff lifting forces
    """

    def setup(self):
        self.add_input("data:system:MTOW:k", val=np.nan, units=None)
        self.add_input("data:system:payload", val=np.nan, units="kg")
        self.add_input("data:propeller:number", val=np.nan, units=None)
        self.add_input(
            "data:scenarios:takeoff:acceleration:z:equivalent", val=np.nan, units="N/kg"
        )

        self.add_output("data:system:MTOW:guess", units="kg")
        self.add_output("data:scenarios:takeoff:force:z", units="N")
        self.add_output("data:scenarios:hover:force:z", units="N")
        self.add_output("data:propeller:force:hover:z", units="N")
        self.add_output("data:propeller:force:takeoff:z", units="N")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs, discrete_inputs=None, discrete_outputs=None):
        k_os = inputs["data:system:MTOW:k"]
        payload = inputs["data:system:payload"]
        N_pro = inputs["data:propeller:number"]
        a_to = inputs["data:scenarios:takeoff:acceleration:z:equivalent"]

        MTOW = k_os * payload
        F_hov = MTOW * g
        F_to = MTOW * (g + a_to*g)
        F_pro_hov = F_hov / N_pro
        F_pro_to = F_to / N_pro

        outputs["data:system:MTOW:guess"] = MTOW
        outputs["data:scenarios:takeoff:force:z"] = F_to
        outputs["data:scenarios:hover:force:z"] = F_hov
        outputs["data:propeller:force:hover:z"] = F_pro_hov
        outputs["data:propeller:force:takeoff:z"] = F_pro_to

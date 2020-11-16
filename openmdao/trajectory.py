import numpy as np
import openmdao.api as om


class Model(om.Group):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("use_torque", types=bool, default=True)
        self.options.declare("model", types=object)

    def setup(self):
        num_points = self.options["num_points"]
        num_simu_points = self.options["num_simu_points"]
        final_time = self.options["final_time"]
        use_torque = self.options["use_torque"]
        model = self.options["model"]

        self.add_subsystem(
            "trajectory",
            Trajectory(
                num_points=num_points,
                num_simu_points=num_simu_points,
                final_time=final_time,
                use_torque=use_torque,
                model=model,
            ),
            promotes=["*"],
        )


class Trajectory(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("use_torque", types=bool, default=True)
        self.options.declare("model", types=object)

    def setup(self):
        # Loading locally the FMU
        self.model = self.options["model"]

        shape = self.options["num_points"]

        if self.options["use_torque"]:
            self.add_input("torque", val=np.nan, shape=shape)
            self.add_output("speed", shape=shape)
        else:
            self.add_input("speed", val=np.nan, shape=shape)
            self.add_output("torque", shape=shape)

        self.add_output("time", shape=shape)
        self.add_output("position", shape=shape)
        self.add_output("rotational_speed", shape=shape)
        self.add_output("acceleration", shape=shape)
        self.add_output("power", shape=shape)
        self.add_output("energy", shape=shape)
        self.add_output("total_energy")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        # Resetting the model is required to re-run from scratch the FMU
        self.model.reset()
        final_time = self.options["final_time"]
        self.model.set("Time", final_time)
        time = np.linspace(0.0, final_time, num=self.options["num_points"])
        time_simu = np.linspace(0.0, final_time, num=self.options["num_simu_points"])
        if self.options["use_torque"]:
            torque = inputs["torque"]
            torque_simu = np.interp(time_simu, time, torque)
            data = np.transpose(np.vstack((time_simu, torque_simu)))
            input_object = ("droneMassPropeller.T", data)
            self.model.set("droneMassPropeller.T", torque_simu[0])
        else:
            self.model.set("droneMassPropeller.xp", inputs["speed"])

        # Simulate
        res = self.model.simulate(
            final_time=final_time,
            input=input_object,
            options={"ncp": self.options["num_simu_points"]-1},
        )

        if self.options["use_torque"]:
            outputs["speed"] = np.interp(time, time_simu, res["droneMassPropeller.xp"])
        else:
            outputs["torque"] = np.interp(time, time_simu, res["droneMassPropeller.T"])

        outputs["time"] = np.interp(time, time_simu, res["time"])
        outputs["position"] = np.interp(time, time_simu, res["droneMassPropeller.x"])
        outputs["rotational_speed"] = np.interp(
            time, time_simu, res["droneMassPropeller.n"]
        )
        outputs["acceleration"] = np.interp(
            time, time_simu, res["droneMassPropeller.der(xp)"]
        )
        outputs["power"] = np.interp(time, time_simu, res["droneMassPropeller.Power"])
        outputs["energy"] = np.interp(time, time_simu, res["NRJ.y"])
        outputs["total_energy"] = res["NRJ.y"][-1]
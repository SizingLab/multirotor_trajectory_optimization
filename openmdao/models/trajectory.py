import os.path as pth
import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

from pyfmi import load_fmu

ressources_folder = "../ressources"
speed_model_file_name = "DroneFMU.DroneOptim_Speed.fmu"
torque_model_file_name = "DroneFMU.DroneOptim_Torque.fmu"

speed_file_path = pth.join(ressources_folder, speed_model_file_name)
torque_file_path = pth.join(ressources_folder, torque_model_file_name)

#Load the FMU
speed_model = load_fmu(speed_file_path) # Model
torque_model = load_fmu(torque_file_path, log_level=4) # Model


num_points = 16
num_simu_points = 500
final_time = 5.
use_torque = True
T_hov = 8.91
torque = np.linspace(T_hov, T_hov, num=num_points)

class Model(om.Group):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("use_torque", types=bool, default=True)
        # self.options.declare("model", types=object, default=model)

    def setup(self):
        num_points = self.options["num_points"]
        num_simu_points = self.options["num_simu_points"]
        final_time = self.options["final_time"]
        use_torque = self.options["use_torque"]
        # model = torque_model

        self.add_subsystem(
            "trajectory",
            Trajectory(
                num_points=num_points,
                num_simu_points=num_simu_points,
                final_time=final_time,
                use_torque=use_torque,
            ),
            promotes=["*"],
        )


class Trajectory(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("use_torque", types=bool, default=True)

    def setup(self):
        self.res = None
        # Loading locally the FMU
        self.model = torque_model

        shape = self.options["num_points"]

        self.add_input("final_time", val=4.0)
        self.add_input("data:propeller:pitch", val=np.nan, units=None)
        self.add_input("data:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:propeller:inertia", val=np.nan, units="kg*m**2")
        self.add_input("data:system:payload:surface", val=np.nan, units="m**2")
        self.add_input("data:system:MTOW:guess", val=np.nan, units="kg")
        self.add_input("data:motor:Kt", val=np.nan, units="N*m/A")
        self.add_input("data:motor:resistance", val=np.nan, units="ohm")
        self.add_input("data:propeller:number", val=np.nan, units=None)
        # self.add_input("data:mission:travel:displacement:number", val=np.nan, units=["s"])
        # self.add_input("data:mission:travel:displacement:height", val=np.nan, units=["m"])

        if self.options["use_torque"]:
            self.add_input("torque", val=torque, shape=shape)
            self.add_output("speed", shape=shape)
        else:
            self.add_input("speed", val=np.nan, shape=shape)
            self.add_output("torque", shape=shape)

        self.add_output("time", shape=shape)
        self.add_output("position", shape=shape)
        self.add_output("rotational_speed", shape=shape)
        self.add_output("acceleration", shape=shape)
        self.add_output("power", shape=shape)
        self.add_output("energy", shape=shape, units="J")
        self.add_output("data:mission:travel:energy", units="J")
        self.add_output("final_position")
        self.add_output("final_speed")
        self.add_output("acc_capacity")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):

        # Resetting the model is required to re-run from scratch the FMU
        self.model.reset()
        self.model.set("droneMassPropeller.beta", float(inputs["data:propeller:pitch"]))
        self.model.set("droneMassPropeller.d", float(inputs["data:propeller:diameter"]))
        self.model.set("droneMassPropeller.J_pro", float(inputs["data:propeller:inertia"]))
        self.model.set("droneMassPropeller.S", float(inputs["data:system:payload:surface"]))
        self.model.set("droneMassPropeller.M", float(inputs["data:system:MTOW:guess"]))
        self.model.set("droneMassPropeller.K_mot", float(inputs["data:motor:Kt"]))
        self.model.set("droneMassPropeller.R_mot", float(inputs["data:motor:resistance"]))
        self.model.set("droneMassPropeller.Np", float(inputs["data:propeller:number"]))

        # final_time = self.options["final_time"]
        final_time = float(inputs["final_time"])
        self.model.set("Time", final_time)
        time = np.linspace(0.0, final_time, num=self.options["num_points"])
        time_simu = np.linspace(0.0, final_time, num=self.options["num_simu_points"])
        if self.options["use_torque"]:
            torque = inputs["torque"]
            torque_simu = np.interp(time_simu, time, torque)
            data = np.transpose(np.vstack((time_simu, torque_simu)))
            input_object = ("T", data)
            self.model.set("T", torque_simu[0])
        else:
            self.model.set("droneMassPropeller.xp", inputs["speed"])

        # Simulate
        res = self.model.simulate(
            final_time=final_time,
            input=input_object,
            options={"ncp": self.options["num_simu_points"]-1},
        )

        self.res = res
        max_torque = np.max(res["droneMassPropeller.T"])
        hover_torque = res["droneMassPropeller.T"][0]

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
        outputs["data:mission:travel:energy"] = res["NRJ.y"][-1]
        outputs["final_position"] = res["droneMassPropeller.x"][-1]
        outputs["final_speed"] = res["droneMassPropeller.xp"][-1]
        outputs["acc_capacity"] = 2 - max_torque/hover_torque


def plot_trajectory(res):

    t = res["time"]
    xpp = res["droneMassPropeller.der(xp)"]
    xp = res["droneMassPropeller.xp"]
    x = res["droneMassPropeller.x"]
    n = res["droneMassPropeller.n"]
    T = res["droneMassPropeller.T"]
    power = res["droneMassPropeller.Power"]

    plt.grid()
    plt.plot(t, xpp, 'k')
    plt.ylabel('Acceleration (m/s²)')
    plt.xlabel('Time (s)')
    plt.show()
    plt.grid()
    plt.plot(t, xp, 'k')
    plt.ylabel('Speed (m/s)')
    plt.xlabel('Time (s)')
    plt.show()
    plt.grid()
    plt.plot(t, x, 'r')
    plt.ylabel('Position (m)')
    plt.xlabel('Time (s)')
    plt.show()
    plt.grid()
    plt.plot(t, n, 'b')
    plt.ylabel('Rotational speed (Hz)')
    plt.xlabel('Time (s)')
    plt.show()
    plt.grid()
    plt.plot(t, T, 'b')
    plt.ylabel('Torque')
    plt.xlabel('Time (s)')
    plt.show()
    plt.grid()
    plt.plot(t, power, 'b')
    plt.ylabel('Total power [W]')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.show()
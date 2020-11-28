import os.path as pth
import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

plt.rc("font", family="serif")
plt.rc("xtick", labelsize="small")
plt.rc("ytick", labelsize="small")

from pyfmi import load_fmu

ressources_folder = "../ressources"
speed_model_file_name = "DroneFMU.DroneOptim_Speed.fmu"
torque_model_file_name = "DroneFMU.DroneOptim_Torque.fmu"

speed_file_path = pth.join(ressources_folder, speed_model_file_name)
torque_file_path = pth.join(ressources_folder, torque_model_file_name)

# Load the FMU
speed_model = load_fmu(speed_file_path)  # Model
torque_model = load_fmu(torque_file_path)  # Model


class Model(om.Group):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("initial_torque", types=float, default=8.91)
        # self.options.declare("model", types=object, default=model)

    def setup(self):
        num_points = self.options["num_points"]
        num_simu_points = self.options["num_simu_points"]
        final_time = self.options["final_time"]
        initial_torque = self.options["initial_torque"]
        # model = torque_model

        self.add_subsystem(
            "trajectory",
            Trajectory(
                num_points=num_points,
                num_simu_points=num_simu_points,
                final_time=final_time,
                initial_torque=initial_torque,
            ),
            promotes=["*"],
        )


class Trajectory(om.ExplicitComponent):
    def initialize(self):
        self.options.declare("num_points", types=int, default=16)
        self.options.declare("num_simu_points", types=int, default=500)
        self.options.declare("final_time", types=float, default=4.0)
        self.options.declare("initial_torque", types=float, default=8.91)
        # self.options.declare("model", types=object, default=model)

    def setup(self):
        self.res = None
        # Loading locally the FMU
        self.model = torque_model

        shape = self.options["num_points"]
        initial_torque = self.options["initial_torque"]
        initial_torque = np.linspace(initial_torque, initial_torque, num=shape)

        self.add_input("data:trajectory:final_time", val=4.0, units="s")
        self.add_input("data:propeller:pitch", val=np.nan, units=None)
        self.add_input("data:propeller:diameter", val=np.nan, units="m")
        self.add_input("data:propeller:inertia", val=np.nan, units="kg*m**2")
        self.add_input("data:system:payload:surface", val=np.nan, units="m**2")
        self.add_input("data:system:MTOW:guess", val=np.nan, units="kg")
        self.add_input("data:motor:Kt", val=np.nan, units="N*m/A")
        self.add_input("data:motor:resistance", val=np.nan, units="ohm")
        self.add_input("data:propeller:number", val=np.nan, units=None)

        self.add_input(
            "data:trajectory:torque", val=initial_torque, shape=shape, units="N*m"
        )

        self.add_output("data:trajectory:time", shape=shape, units="s")
        # self.add_output("data:trajectory:position", shape=shape, units="m")
        # self.add_output("data:trajectory:speed", shape=shape, units="m/s")
        # self.add_output("data:trajectory:rotational_speed", shape=shape, units="rad/s")
        self.add_output("data:trajectory:rotational_speed:max", units="rad/s")
        self.add_output("data:trajectory:torque:max", units="N*m")
        # self.add_output("data:trajectory:acceleration", shape=shape, units="m/s**2")
        # self.add_output("data:trajectory:power", shape=shape, units="W")
        # self.add_output("data:trajectory:energy", shape=shape, units="J")
        self.add_output("data:mission:travel:energy", units="J")
        self.add_output("data:trajectory:final_position", units="m")
        self.add_output("data:trajectory:final_speed", units="m/s")
        # self.add_output("data:trajectory:acc_capacity", units="m/s**2")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):

        # Resetting the model is required to re-run from scratch the FMU
        self.model.reset()

        self.model.set("droneMassPropeller.beta", float(inputs["data:propeller:pitch"]))
        self.model.set("droneMassPropeller.d", float(inputs["data:propeller:diameter"]))
        self.model.set(
            "droneMassPropeller.J_pro", float(inputs["data:propeller:inertia"])
        )
        self.model.set(
            "droneMassPropeller.S", float(inputs["data:system:payload:surface"])
        )
        self.model.set("droneMassPropeller.M", float(inputs["data:system:MTOW:guess"]))
        self.model.set("droneMassPropeller.K_mot", float(inputs["data:motor:Kt"]))
        self.model.set(
            "droneMassPropeller.R_mot", float(inputs["data:motor:resistance"])
        )
        self.model.set("droneMassPropeller.Np", float(inputs["data:propeller:number"]))

        # final_time = self.options["final_time"]
        final_time = float(inputs["data:trajectory:final_time"])
        self.model.set("Time", final_time)
        time = np.linspace(0.0, final_time, num=self.options["num_points"])
        time_simu = np.linspace(0.0, final_time, num=self.options["num_simu_points"])

        torque = inputs["data:trajectory:torque"]
        torque_simu = np.interp(time_simu, time, torque)
        data = np.transpose(np.vstack((time_simu, torque_simu)))
        input_object = ("T", data)
        self.model.set("T", torque_simu[0])

        # Simulate
        res = self.model.simulate(
            final_time=final_time,
            input=input_object,
            options={"ncp": self.options["num_simu_points"] - 1},
        )

        self.res = res

        # outputs["data:trajectory:speed"] = np.interp(time, time_simu, res["droneMassPropeller.xp"])
        outputs["data:trajectory:time"] = np.interp(time, time_simu, res["time"])
        # outputs["data:trajectory:position"] = np.interp(time, time_simu, res["droneMassPropeller.x"])
        outputs["data:trajectory:rotational_speed:max"] = np.max(
            res["droneMassPropeller.n"]
        )
        outputs["data:trajectory:torque:max"] = np.max(res["droneMassPropeller.T"])

        # outputs["data:trajectory:rotational_speed"] = np.interp(
        #     time, time_simu, res["droneMassPropeller.n"]
        # )
        # outputs["data:trajectory:acceleration"] = np.interp(
        #     time, time_simu, res["droneMassPropeller.der(xp)"]
        # )

        # outputs["data:trajectory:power"] = np.interp(time, time_simu, res["droneMassPropeller.Power"])
        # outputs["data:trajectory:energy"] = np.interp(time, time_simu, res["NRJ.y"])
        outputs["data:mission:travel:energy"] = res["NRJ.y"][-1]
        outputs["data:trajectory:final_position"] = res["droneMassPropeller.x"][-1]
        outputs["data:trajectory:final_speed"] = res["droneMassPropeller.xp"][-1]
        # outputs["data:trajectory:acc_capacity"] = 2 - max_torque/hover_torque


def plot_trajectory(res):

    t = res["time"]
    xpp = res["droneMassPropeller.der(xp)"]
    xp = res["droneMassPropeller.xp"]
    x = res["droneMassPropeller.x"]
    n = res["droneMassPropeller.n"]
    T = res["droneMassPropeller.T"]
    power = res["droneMassPropeller.Power"]
    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(
        3, 2, sharex="col", figsize=(10, 6)
    )

    # Acceleration
    ax1.plot(t, xpp, "k")
    ax1.set_ylabel("Acceleration [m/s²]")
    # ax1.set_xlabel("Time [s]")
    ax1.grid()

    # Speed
    ax3.plot(t, xp, "k")
    ax3.set_ylabel("Speed [m/s]")
    # ax2.set_xlabel("Time [s]")
    ax3.grid()

    # Position
    ax5.plot(t, x, "k")
    ax5.set_ylabel("Position [m]")
    ax5.set_xlabel("Time [s]")
    ax5.grid()

    # Motor speed
    ax2.plot(t, n, "k")
    ax2.set_ylabel("Motor rotational speed [Hz]")
    # ax4.set_xlabel("Time [s]")
    ax2.grid()

    # Motor torque
    ax4.plot(t, T, "k")
    ax4.set_ylabel("Motor torque [N.m]")
    # ax5.set_xlabel("Time [s]")
    ax4.grid()

    # Total electric  power
    ax6.plot(t, power * 1e-3, "k")
    ax6.set_ylabel("Total electric power [kW]")
    ax6.set_xlabel("Time [s]")
    ax6.grid()

    fig.align_ylabels()
    fig.tight_layout()
    # plt.legend()
    plt.show()

import numpy as np
import matplotlib.pyplot as plt
import openmdao.api as om

plt.rc("font", family="serif")
plt.rc("xtick", labelsize="small")
plt.rc("ytick", labelsize="small")

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
        self.res = None
        # Loading locally the FMU
        self.model = self.options["model"]

        shape = self.options["num_points"]

        self.add_input("final_time", val=self.options["final_time"])

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
        self.add_output("final_position")
        self.add_output("final_speed")
        self.add_output("acc_capacity")

        self.declare_partials("*", "*", method="fd")

    def compute(self, inputs, outputs):
        # Resetting the model is required to re-run from scratch the FMU
        self.model.reset()
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
            options={"ncp": self.options["num_simu_points"] - 1},
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
        outputs["total_energy"] = res["NRJ.y"][-1]
        outputs["final_position"] = res["droneMassPropeller.x"][-1]
        outputs["final_speed"] = res["droneMassPropeller.xp"][-1]
        outputs["acc_capacity"] = 2 - max_torque / hover_torque


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

    return fig

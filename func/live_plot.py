
import matplotlib.pyplot as plt

class MotherStatePlotter:
    def __init__(self, world):
        self.world = world

        self.state_names = [
            "energy",
            "fatigue",
            "bonding",
            "fear_threat",
            "stress",
            "closeness_child",
            "OT",
            "CORT",
        ]

        plt.ion()


        self.fig, self.axes = plt.subplots(4, 2, figsize=(7, 7 ))
        self.axes = self.axes.flatten()

        self.lines = {}  # (mother_id, state_name) -> line
        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+0+0")

        for ax, state_name in zip(self.axes, self.state_names):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.set_ylim(0, 100)
            ax.grid(True)

        for m in self.world.mothers:
            for ax, state_name in zip(self.axes, self.state_names):
                line, = ax.plot([], [], label=m.id)
                self.lines[(m.id, state_name)] = line

        for ax in self.axes:
            ax.legend(loc="upper right")

        self.fig.tight_layout()

    def update(self):
        x = list(self.world.tick_history)
        if not x:
            return

        for m in self.world.mothers:
            hist = self.world.mother_history[m.id]

            for ax, state_name in zip(self.axes, self.state_names):
                y = list(hist[state_name])
                line = self.lines[(m.id, state_name)]
                line.set_xdata(x)
                line.set_ydata(y)
                ax.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


import matplotlib.pyplot as plt

class MotherMotivationPlotter:
    def __init__(self, world):
        self.world = world

        plt.ion()

        self.fig, self.axes = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
        self.ax_values = self.axes[0]
        self.ax_active = self.axes[1]

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.ax_values.set_title("Motivation values")
        self.ax_values.set_ylabel("Value")
        self.ax_values.set_ylim(0, 100)
        self.ax_values.grid(True)

        self.ax_active.set_title("Motivation activation")
        self.ax_active.set_xlabel("Tick")
        self.ax_active.set_ylabel("Activation")
        self.ax_active.set_ylim(-0.05, 1.05)
        self.ax_active.grid(True)

        self.mot_keys = ["Forage", "Care", "Self", "Protect"]

        # choose your fixed colors here
        self.colors = {
            "Forage": "gold",
            "Care": "limegreen",
            "Self": "deepskyblue",
            "Protect": "red",
        }

        self.lines_values = {}   # (mother_id, motivation_name) -> line
        self.lines_active = {}   # (mother_id, motivation_name) -> line

        for m in self.world.mothers:
            for mot in self.mot_keys:
                line_val, = self.ax_values.plot(
                    [], [], label=mot, color=self.colors[mot]
                )
                self.lines_values[(m.id, mot)] = line_val

                line_act, = self.ax_active.plot(
                    [], [], label=mot, color=self.colors[mot]
                )
                self.lines_active[(m.id, mot)] = line_act

            # if multiple mothers, only first mother's labels should appear in legend
            # break

        self.ax_values.legend(loc="upper right", ncol=4)
        self.ax_active.legend(loc="upper right", ncol=4)

        self.fig.tight_layout()

    # def update(self, mother_id=None):
    #     x = list(self.world.tick_history)
    #     if not x:
    #         return

    #     # default: plot first mother
    #     if mother_id is None:
    #         if not self.world.mothers:
    #             return
    #         mother_id = self.world.mothers[0].id

    #     hist = self.world.mother_history[mother_id]

    #     for mot in self.mot_keys:
    #         y_val = list(hist[f"mot_{mot.lower()}"])
    #         self.lines_values[(mother_id, mot)].set_xdata(x)
    #         self.lines_values[(mother_id, mot)].set_ydata(y_val)

    #         y_act = list(hist[f"sel_{mot.lower()}"])
    #         self.lines_active[(mother_id, mot)].set_xdata(x)
    #         self.lines_active[(mother_id, mot)].set_ydata(y_act)

    #     self.ax_values.set_xlim(max(0, x[0]), max(10, x[-1]))
    #     self.ax_active.set_xlim(max(0, x[0]), max(10, x[-1]))

    #     self.fig.canvas.draw()
    #     self.fig.canvas.flush_events()
    #     plt.pause(0.001)

    def update(self, mother_id=None):

        x = list(self.world.tick_history)
        if not x:
            return

        # if no mother specified, use first alive mother
        if mother_id is None:
            if not self.world.mothers:
                return
            mother_id = self.world.mothers[0].id

        hist = self.world.mother_history.get(mother_id)
        if hist is None:
            return

        for mot in self.mot_keys:

            y_val = list(hist[f"mot_{mot.lower()}"])
            self.lines_values[(mother_id, mot)].set_xdata(x)
            self.lines_values[(mother_id, mot)].set_ydata(y_val)

            y_act = list(hist[f"sel_{mot.lower()}"])
            self.lines_active[(mother_id, mot)].set_xdata(x)
            self.lines_active[(mother_id, mot)].set_ydata(y_act)

        self.ax_values.set_xlim(max(0, x[0]), max(10, x[-1]))
        self.ax_active.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

class ChildStatePlotter:
    def __init__(self, world):
        self.world = world

        self.state_names = [
            "hunger",
            "warmth",
            "injury",
            # "carried",
        ]

        plt.ion()

        self.fig, self.axes = plt.subplots(1, 3, figsize=(8, 3))
        self.axes = self.axes.flatten()

        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+880+0")

        self.lines = {}

        for ax, state_name in zip(self.axes, self.state_names):
            ax.set_title(state_name)
            ax.set_xlabel("Tick")
            ax.set_ylabel("Value")
            ax.grid(True)

            if state_name != "carried":
                ax.set_ylim(0, 100)
            else:
                ax.set_ylim(-0.1, 1.1)

        for c in self.world.children:
            for ax, state_name in zip(self.axes, self.state_names):
                line, = ax.plot([], [], label=c.id)
                self.lines[(c.id, state_name)] = line

        for ax in self.axes:
            ax.legend()

        self.fig.tight_layout()

    def update(self):
        x = list(self.world.tick_history)
        if not x:
            return

        for c in self.world.children:
            hist = self.world.child_history[c.id]

            for ax, state_name in zip(self.axes, self.state_names):
                y = list(hist[state_name])
                x_local = x[-len(y):] if len(y) > 0 else []
                line = self.lines[(c.id, state_name)]
                line.set_xdata(x_local)
                line.set_ydata(y)

                ax.set_xlim(max(0, x[0]), max(10, x[-1]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

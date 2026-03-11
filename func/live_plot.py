
import matplotlib.pyplot as plt

class MotherStatePlotter:
    def __init__(self, world):
        self.world = world

        self.state_names = [
            "energy",
            # "fatigue",
            # "bonding",
            "fear_threat",
            "stress",
            # "closeness_child",
            "OT",
            "CORT",
        ]

        plt.ion()


        self.fig, self.axes = plt.subplots(4, 2, figsize=(8, 8))
        self.axes = self.axes.flatten()

        self.lines = {}  # (mother_id, state_name) -> line
        manager = plt.get_current_fig_manager()
        manager.window.wm_geometry("+500+0")

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
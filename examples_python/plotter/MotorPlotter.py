import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class LivePlot:
    def __init__(self, num_lines=1, interval=10,xlim=(0,10),ylim=(0,10)):
        self.fig, self.ax = plt.subplots()
        self.lines = {}
        self.num_lines = num_lines
        self.interval = interval

        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.anim = FuncAnimation(self.fig, self.update, interval=self.interval, blit=True)

    def update(self, frame):
        # Update line data
        for line_name, line in self.lines.items():
            x_data, y_data = line["data"]
            line["line"].set_data(x_data, y_data)
        return list(line["line"] for line in self.lines.values())

    def add_line(self, name, x_data=None, y_data=None):
        line, = self.ax.plot([], [])
        self.lines[name] = {"line": line, "data": ([], [])}
        self.num_lines += 1
        line.set_label(name)
        self.ax.legend()

        if x_data is not None and y_data is not None:
            self.set_data(name, x_data, y_data)

    def set_data(self, name, x_data, y_data):
        if(self.lines.get(name) == None):
            self.add_line(name, x_data, y_data)
        line = self.lines.get(name)
        if line is not None:
            line["data"] = (x_data, y_data)

    def append_data(self, name, x, y):
        if(self.lines.get(name) == None):
            self.add_line(name, x, y)
        line = self.lines.get(name)
        if line is not None:
            x_data, y_data = line["data"]
            x_data.append(x)
            y_data.append(y)
    def show(self,time=0.001):
        plt.pause(time)
    def isShowing(self):
        return plt.fignum_exists(self.fig.number)
    def close(self):
        plt.close(self.fig.number)

# # Example usage
# plot = LivePlot(num_lines=1)  # Create a LivePlot object with 1 line
# plot.add_line("Line 1")  # Add a line with name "Line 1"
# plot.add_line("Line 2")  # Add a line with name "Line 2"
# plot.add_line("Line 3")  # Add a line with name "Line 3"
# for i in range(10000):
#     plot.append_data("Line 1", x=1+i, y=2)
#     plot.append_data("Line 2", x=2+i, y=4)
#     plot.append_data("Line 3", x=3+i, y=6)
#     plot.show()
#     time.sleep(0.001)

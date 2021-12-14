from tkinter import *
from tkinter.colorchooser import askcolor
import pandas as pd


class Paint(object):

    DEFAULT_PEN_SIZE = 5.0
    DEFAULT_COLOR = "black"

    def __init__(self):
        self.root = Tk()

        self.pen_button = Button(self.root, text="pen", command=self.use_pen)
        self.pen_button.grid(row=0, column=0)

        self.brush_button = Button(self.root, text="brush", command=self.use_brush)
        self.brush_button.grid(row=0, column=1)

        self.color_button = Button(self.root, text="color", command=self.choose_color)
        self.color_button.grid(row=0, column=2)

        self.eraser_button = Button(self.root, text="eraser", command=self.use_eraser)
        self.eraser_button.grid(row=0, column=3)

        self.choose_size_button = Scale(self.root, from_=1, to=10, orient=HORIZONTAL)
        self.choose_size_button.grid(row=0, column=4)

        self.save_button = Button(self.root, text="Save", command=self.save)
        self.save_button.grid(row=0, column=5)

        self.c = Canvas(self.root, bg="white", width=1500, height=1500)
        self.c.grid(row=1, columnspan=6)

        self.positions = pd.DataFrame()

        self.setup()
        self.root.mainloop()

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.line_width = self.choose_size_button.get()
        self.color = self.DEFAULT_COLOR
        self.eraser_on = False
        self.active_button = self.pen_button
        self.c.bind("<B1-Motion>", self.paint)
        self.c.bind("<ButtonRelease-1>", self.reset)

    def use_pen(self):
        self.activate_button(self.pen_button)

    def use_brush(self):
        self.activate_button(self.brush_button)

    def choose_color(self):
        self.eraser_on = False
        self.color = askcolor(color=self.color)[1]

    def use_eraser(self):
        self.activate_button(self.eraser_button, eraser_mode=True)

    def activate_button(self, some_button, eraser_mode=False):
        self.active_button.config(relief=RAISED)
        some_button.config(relief=SUNKEN)
        self.active_button = some_button
        self.eraser_on = eraser_mode

    def save(self):
        # move to top left
        self.positions["pos_x"] -= self.positions["pos_x"].min()
        self.positions["pos_y"] -= self.positions["pos_y"].min()

        # mirror
        self.positions["pos_y"] = (
            self.positions["pos_y"].max() - self.positions["pos_y"]
        )
        # scale to [0, 1]
        self.positions[["pos_x", "pos_y"]] /= self.positions[["pos_x", "pos_y"]].max()

        self.positions.to_csv("trajectory.csv", index=False)
        print(self.positions)

    def paint(self, event):
        self.line_width = self.choose_size_button.get()
        paint_color = "white" if self.eraser_on else self.color
        if self.old_x and self.old_y:
            self.c.create_line(
                self.old_x,
                self.old_y,
                event.x,
                event.y,
                width=self.line_width,
                fill=paint_color,
                capstyle=ROUND,
                smooth=TRUE,
                splinesteps=36,
            )

        self.positions = self.positions.append(
            {
                "pos_x": event.x,
                "pos_y": event.y,
                "color_r": 255,
                "color_g": 0,
                "color_b": 0,
            },
            ignore_index=True,
        )

        self.old_x = event.x
        self.old_y = event.y

    def reset(self, event):
        self.old_x, self.old_y = None, None


if __name__ == "__main__":
    Paint()

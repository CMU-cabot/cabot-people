import sys
from std_msgs.msg import ColorRGBA

class Colors(object):
    # This class assigns colors to the markers for visualization purposes

    def __init__(self, values=None):
        if values is None:
            values = [0.0, 1.0]
        for v in values:
            if (not isinstance(v, float)) or (v < 0.0) or (v > 1.0):
                raise Exception("Color values must be 0.0 - 1.0")
                sys.exit(0)
        self.color_palette = self._generate_palette(values)
        self.num_colors = len(self.color_palette)

    def _generate_palette(self, values):
        palette = []
        for i in values:
            for j in values:
                for k in values:
                    palette.append([i, j, k])
        return palette

    def get_color(self, seed, fixed=None):
        if fixed is None:
            idx = seed % self.num_colors
            color = self.color_palette[idx]
        else:
            if not (fixed >= 0 and fixed < self.num_colors):
                print("Available colors are")
                print(self.color_palette)
                raise Exception("Color indexes are from 0 to {}".format(self.num_colors))
                sys.exit(0)
            color = self.color_palette[fixed]
        color_msg = ColorRGBA()
        color_msg.r = color[0]
        color_msg.g = color[1]
        color_msg.b = color[2]
        color_msg.a = 1.0
        return color_msg
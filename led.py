import neopixel
import board
import re


pixel_pin = board.D21
num_pixels = 30
dimmer = 100

# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=0.2, auto_write=False, pixel_order=ORDER
)

last_color = [255, 255, 255]

class LedControl:
    def __init__(self):
        self._running = True

    def terminate(self):
        self._running = False


    def hex_to_rgb(self, hex):
      rgb = []
      for i in (0, 2, 4):
        decimal = int(hex[i:i+2], 16)
        rgb.append(decimal)

      return tuple(rgb)



    def setLightColor(self, color_hex):
        global last_color
        last_color = self.hex_to_rgb(color_hex)
        pixels.fill(last_color)
        # Uncomment this line if you have RGBW/GRBW NeoPixels
        # pixels.fill((255, 0, 0, 0))
        pixels.show()

    def setLightDimmer(self, dimmer):
        global last_color
        new_value = int(re.findall(r"'(.*?)'", dimmer)[0])/100
        dimmed_color = [int(last_color[0]*new_value), int(last_color[1]*new_value), int(last_color[2]*new_value)]
        pixels.fill(dimmed_color)
        pixels.show()

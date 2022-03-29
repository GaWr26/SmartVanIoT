import neopixel
import board
import re
import time


pixel_pin = board.D21
num_pixels = 30
dimmer = 100

# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.GRB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=1, auto_write=False, pixel_order=ORDER
)

last_color = [255, 255, 255]

loopEffect = False

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
        global loopEffect
        loopEffect = False
        last_color = self.hex_to_rgb(color_hex)
        pixels.fill(last_color)
        # Uncomment this line if you have RGBW/GRBW NeoPixels
        # pixels.fill((255, 0, 0, 0))
        pixels.show()

    def setLightDimmer(self, dimmer):
        global last_color
        global loopEffect
        loopEffect = False
        new_value = int(re.findall(r"'(.*?)'", dimmer)[0])/100
        dimmed_color = [int(last_color[0]*new_value), int(last_color[1]*new_value), int(last_color[2]*new_value)]
        pixels.fill(dimmed_color)
        pixels.show()

    def wheel(self, pos):
    # Input a value 0 to 255 to get a color value.
    # The colours are a transition r - g - b - back to r.
        if pos < 0 or pos > 255:
            r = g = b = 0
        elif pos < 85:
            r = int(pos * 3)
            g = int(255 - pos * 3)
            b = 0
        elif pos < 170:
            pos -= 85
            r = int(255 - pos * 3)
            g = 0
            b = int(pos * 3)
        else:
            pos -= 170
            r = 0
            g = int(pos * 3)
            b = int(255 - pos * 3)
        return (r, g, b) if ORDER in (neopixel.RGB, neopixel.GRB) else (r, g, b, 0)


    def rainbow_cycle(self, wait):
        global loopEffect
        loopEffect = True
        while loopEffect:
            for j in range(255):
                for i in range(num_pixels):
                    pixel_index = (i * 256 // num_pixels) + j
                    pixels[i] = self.wheel(pixel_index & 255)
                pixels.show()
                time.sleep(wait)

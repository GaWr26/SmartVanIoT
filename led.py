import neopixel
import board
import re
import time
import threading

from adafruit_led_animation.animation.solid import Solid
from adafruit_led_animation.animation.colorcycle import ColorCycle
from adafruit_led_animation.animation.blink import Blink
from adafruit_led_animation.animation.comet import Comet
from adafruit_led_animation.animation.chase import Chase
from adafruit_led_animation.animation.pulse import Pulse
from adafruit_led_animation.sequence import AnimationSequence
from adafruit_led_animation.color import (
    PURPLE,
    WHITE,
    AMBER,
    JADE,
    TEAL,
    PINK,
    MAGENTA,
    ORANGE,
)


pixel_pin = board.D21
num_pixels = 300
dimmer = 100

# For RGBW NeoPixels, simply change the ORDER to RGBW or GRBW.
ORDER = neopixel.RGB

pixels = neopixel.NeoPixel(
    pixel_pin, num_pixels, brightness=1, auto_write=False, pixel_order=ORDER
)

solid = Solid(pixels, color=PINK)
blink = Blink(pixels, speed=0.5, color=JADE)
colorcycle = ColorCycle(pixels, speed=0.4, colors=[MAGENTA, ORANGE, TEAL])
chase = Chase(pixels, speed=0.1, color=WHITE, size=3, spacing=6)
comet = Comet(pixels, speed=0.01, color=PURPLE, tail_length=10, bounce=True)
pulse = Pulse(pixels, speed=0.1, color=AMBER, period=3)

last_color = [255, 255, 255]

loopEffect = False

class LED(threading.Thread):
    def __init__(self, parent=None):
        self.parent = parent
        super(LED, self).__init__()

    def target_with_callback(self):
        self.method()
        if self.callback is not None:
            self.callback(*self.callback_args)

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
        print("set dimmer " + str(new_value) + " " + str(dimmed_color))


    def toggleAnimation(self, id, switch):
        print("toggle Animation: " + str(id) + " " + str(switch))
        new_value = re.findall(r"'(.*?)'", str(switch))[0]
        if new_value == "on":
            animate = True
        else:
            animate = False

        animations = AnimationSequence(
            solid,
            blink,
            colorcycle,
            chase,
            comet,
            pulse,
            advance_interval=5,
            auto_clear=True,
        )

        while animate:
            animations.animate()

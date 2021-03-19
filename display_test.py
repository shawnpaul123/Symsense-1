import sys
sys.path.insert(1, "./lib") # Adds lib folder in this directory to sys
import os
import logging
from waveshare_epd import epd2in9d
import time
from PIL import Image,ImageDraw,ImageFont
import traceback

# set folder paths
picdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pic')
libdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lib')

# set fonts
font24 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 24)
font16 = ImageFont.truetype(os.path.join(picdir, 'Font.ttc'), 16)

# init screen + clear 
epd = epd2in9d.EPD()
epd.init()
epd.Clear(0xFF)

#  start drawing image
image = Image.new('1', (epd.height, epd.width), 255)  # 255: clear the frame
draw = ImageDraw.Draw(image)
draw.text((0, 0), 'line 1, font 24', font = font24, fill = 0)
draw.text((0, 20), 'line 2', font = font24, fill = 0)
draw.text((0, 40), 'line 3', font = font24, fill = 0)
draw.text((0, 60), 'line 4', font = font24, fill = 0)
draw.text((0, 85), 'anna is awesome, font 16', font = font16, fill = 0)
draw.text((0, 100), 'line 6', font = font24, fill = 0)

# draw image and hold for 5 sec
epd.display(epd.getbuffer(image)) 
time.sleep(5)

# clear everything + put display to sleep
epd.Clear(0xFF) 
epd.sleep()
time.sleep(1)
epd.Dev_exit()
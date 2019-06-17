from microbit import *
import random

print("Buttons")
# Press A to start.
while True:
    if button_a.was_pressed():
        break
    else:
        display.show(Image.ARROW_W)
        sleep(200)
        display.clear()
        sleep(200)
while True:
    if button_b.was_pressed():
        break
    else:
        display.show(Image.ARROW_E)
        sleep(200)
        display.clear()
        sleep(200)

print("Images")
# Grab all the built in images.
images = [getattr(Image, img) for img in dir(Image)
          if type(getattr(Image, img)) == Image]
# ... and cycle through them on the display.
pause = 1000
for img in images:
    display.show(img)
    sleep(pause)
    pause -= 50
    if pause < 100:
        pause = 100
display.clear()

print("Light")
button_a.was_pressed()
while not button_a.was_pressed():
    display.scroll(display.read_light_level())
sleep(500)

print("Temperature")
button_a.was_pressed()
while not button_a.was_pressed():
    display.scroll(temperature())
sleep(500)

# Aural testing of the accelerometer.
print("Accelerometer")
print("X")
button_a.was_pressed()
while not button_a.was_pressed():
    display.scroll(accelerometer.get_x())
sleep(500)
print("Y")
while not button_a.was_pressed():
    display.scroll(accelerometer.get_y())
sleep(500)
print("Z")
while not button_a.was_pressed():
    display.scroll(accelerometer.get_z())

print("Gestures")
while not button_a.was_pressed():
  gestures = accelerometer.get_gestures()
  for i in range(0, len(gestures)):
    display.scroll(gestures[i])

# Pixel brightness.
print("Display Random")
dots = [ [0]*5, [0]*5, [0]*5, [0]*5, [0]*5 ]
while not button_a.is_pressed():
    dots[random.randrange(5)][random.randrange(5)] = 9
    for i in range(5):
        for j in range(5):
            display.set_pixel(i, j, dots[i][j])
            dots[i][j] = max(dots[i][j] - 1, 0)
    sleep(50)


# Finished!
display.scroll("Finished!")
display.show(Image.HAPPY)
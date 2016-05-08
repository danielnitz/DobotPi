import RPi.GPIO as GPIO, time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(18,GPIO.OUT)
p = GPIO.PWM(16,500)

def SpinMotor(direction,numSteps):
    GPIO.output(18,direction)
    while numSteps > 0:
        p.start(1)
        time.sleep(.01)
        numSteps -= 1
    p.stop()
    GPIO.cleanup()
    return True

while(True):
    SpinMotor(True, 800)
    time.sleep(.5)


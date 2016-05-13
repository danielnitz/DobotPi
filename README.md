# DobotPi - Control Dobot on the Raspberry Pi 3
This software allows you to control a Dobot robotic arm with a raspberry pi 3. There are numerous advantages to controlling the Dobot with a raspberry pi as opposed to an arduino, FPGA, or other electronics that requires movement data to be transmitted serially

To Do (Hardware and software to do list, my personal opinions):

-Install limit switches on the Dobot and implement the code to control them. Could use a photointerrupter like Max (open-dobot) or some hall effect limit switches that use magnets (or combination). Either is probably better than a manual limit switch. All are only a few dollars each. Will likely need to 3d print some mounts for these.

-Buy and install NEMA 17 stepper motors that have greater torque (for increased payload ability). Unsure how much these will cost. probably around $20-40 each. The stepper motors that came with the Dobot have nice screws on the back that rotate with the motor and would enable one to mount a rotary encoder on the rear, perhaps needing a 3d printed adapter.

-Buy and install rotary encoders on the stepper motors (to keep track of the actual steps taken and avoid missed steps). These can be used to detect if someone hits the arm for example, and the robot can then account for the missed steps. These will be ~$20-40 each depending on which one you get.

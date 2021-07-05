"""Coltrol_prueba_movimiento controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Keyboard

# create the Robot instance.
robot = Robot()
keyboard=Keyboard()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
keyboard.enable(timestep)

camera = robot.getDevice("camera");
camera.enable(timestep);


wheels=[]
wheelsNames=['left_boggie_directional','left_rocker_directional',
'der_rocker_directional','der_boggie_directional','wheel_boggie_left',
'middle_wheel_left','wheel_rocker_left','wheel_boggie_der',
'middle_wheel_der','wheel_rocker_der']

for i in range(10):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    giro=0.0
    vel_lin=0.0
    key=keyboard.getKey()
    #print(key)
    if key==87:
        vel_lin=10.0
    if key==83:
        vel_lin=-10.0
    if key== 65:
        giro=1.0
    if key==68:
        giro=-1.0
    print(vel_lin)
    print(giro)
    
    wheels[0].setVelocity(-giro)
    wheels[1].setVelocity(giro)
    wheels[2].setVelocity(giro)
    wheels[3].setVelocity(-giro)
    wheels[4].setVelocity(vel_lin)
    wheels[5].setVelocity(vel_lin)
    wheels[6].setVelocity(vel_lin)
    wheels[7].setVelocity(vel_lin)
    wheels[8].setVelocity(vel_lin)
    wheels[9].setVelocity(vel_lin)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

from commands2.button import JoystickButton, CommandXboxController
from wpilib.interfaces import GenericHID

class driverstation:
    joysticks = GenericHID(0)
    panelA = GenericHID(1)
    panelB = GenericHID(2)

    leftJoy = JoystickButton(joysticks, 1)
    rightJoy = JoystickButton(joysticks, 2)
    leftJoyY = lambda: -driverstation.joysticks.getRawAxis(1)
    leftJoyX = lambda: -driverstation.joysticks.getRawAxis(0)
    rightJoyX = lambda: -driverstation.joysticks.getRawAxis(2)
    shoot = JoystickButton(panelA, 1)
    amp = JoystickButton(panelA, 2)
    extraA =  JoystickButton(panelA, 3)
    trap =  JoystickButton(panelA, 4)
    extraB =  JoystickButton(panelA, 6)
    ground =  JoystickButton(panelA, 7)
    source =  JoystickButton(panelA, 8)
    rollers =  JoystickButton(panelA, 9)
    drop =  JoystickButton(panelA, 10)
    extraC =  JoystickButton(panelA,11)
    climb =  JoystickButton(panelB, 2)
    ohShit =  JoystickButton(panelB,   3)
    rightest =  JoystickButton(panelB, 4)
    right =  JoystickButton(panelB, 5)
    left =  JoystickButton(panelB, 6)
    gyro =  JoystickButton(panelB, 7)
    leftest =  JoystickButton(panelB, 8)

    deadband = 0.05

class handheld:
    controller = CommandXboxController(5)
    
    leftY =  lambda: -handheld.controller.getRawAxis(1)
    leftX =  lambda: -handheld.controller.getRawAxis(0)
    rightX = lambda: -handheld.controller.getRawAxis(4)
    A = controller.a()
    B = controller.b()
    X = controller.x()
    Y = controller.y()
    RB = controller.rightBumper()
    LB = controller.leftBumper()
    LT = controller.leftTrigger(0.5)
    RT = controller.rightTrigger(0.5)

    deadband = 0.09
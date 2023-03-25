import commands2
import rev
import wpilib

from map import ClawConstants


class Claw(commands2.SubsystemBase):
    def __init__(self):
        commands2.SubsystemBase.__init__(self)

        self.motor = rev.CANSparkMax(ClawConstants.MOTOR_PORT, rev.CANSparkMax.MotorType.kBrushless)

        self._config_motors()

        # An AnalogPotentiometer class can also represent an analog distance sensor
        self.distance_sense = wpilib.AnalogPotentiometer(ClawConstants.ANALOG_DISTANCE_SENSOR_PORT, 1023)
        wpilib.SmartDashboard.putData("Claw Distance Sensor", self.distance_sense)

    def _config_motors(self):
        self.motor.setSmartCurrentLimit(10)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.motor.setOpenLoopRampRate(0.5)

    def periodic(self) -> None:
        pass

    def intake(self):
        self.motor.set(1)

    def outtake(self):
        self.motor.set(-1)

    def stop_command(self):
        return commands2.InstantCommand(self.motor.stopMotor, self)

    def intake_command(self):
        return commands2.RunCommand(self.intake, self)

    def outtake_command(self):
        return commands2.RunCommand(self.outtake, self)

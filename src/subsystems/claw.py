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

    def periodic(self) -> None:
        pass

    def intake(self):
        self.motor.set(1)

    def outtake(self):
        self.motor.set(-1)

    def stop(self):
        self.motor.set(0)

    def intake_command(self):
        return commands2.InstantCommand(self.intake).withTimeout(1).andThen(self.stop)

    def outtake_command(self):
        return commands2.InstantCommand(self.outtake).withTimeout(1).andThen(self.stop)

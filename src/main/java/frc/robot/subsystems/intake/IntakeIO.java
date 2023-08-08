package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    void updateInputs(IntakeInputs inputs);

    double getAngle();

    void setAngle(double angle);

    double getSpinMotorPower();

    void setSpinMotorPower(double power);

    @AutoLog
    class IntakeInputs{
        public double angle = 0;
        public double angleSetpoint = 0;
        public double angleMotorVelocity = 0;
        public double angleMotorAppliedVoltage = 0;

        public double spinMotorAppliedVoltage = 0;
    }
}

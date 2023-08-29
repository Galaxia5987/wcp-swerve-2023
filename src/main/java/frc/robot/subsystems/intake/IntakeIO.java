package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    void updateInputs(IntakeInputs inputs);

    double getAngle();

    void setAngle(double angle);

    default double getSpinMotorVelocity(){
        return 0;
    }

    void setSpinMotorPower(double power);

    double getAngleMotorCurrent();

    default void resetEncoder(double angle){}

    @AutoLog
    class IntakeInputs{
        public double angle = 0;
        public double angleSetpoint = 0;
        public double angleMotorVelocity = 0;
        public double angleMotorAppliedVoltage = 0;

        public double spinMotorAppliedVoltage = 0;
        public double spinMotorVelocity = 0;
    }
}

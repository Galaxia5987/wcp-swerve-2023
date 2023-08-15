package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO{
    private final FlywheelSim angleMotor;
    private final FlywheelSim spinMotor;

    public IntakeIOSim(){
        angleMotor = new FlywheelSim(
                DCMotor.getFalcon500(1),
                IntakeConstants.GEAR_RATIO,
                IntakeConstants.ANGLE_MOTOR_MOMENT_OF_INERTIA
        );

        spinMotor = new FlywheelSim(
                DCMotor.getNEO(1),
                1,
                IntakeConstants.SPIN_MOTOR_MOMENT_OF_INERTIA
        );
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        angleMotor.update(0.02);
        spinMotor.update(0.02);


        inputs.angle = currentAngle.get();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angleMotorAppliedVoltage = angleMotorAppliedVoltage;

        inputs.spinMotorAppliedVoltage = spinMotorAppliedVoltage;
    }

    @Override
    public double getAngle() {
    }

    @Override
    public void setAngle(double angle) {

    }

    @Override
    public double getSpinMotorPower() {
        return 0;
    }

    @Override
    public void setSpinMotorPower(double power) {

    }
}

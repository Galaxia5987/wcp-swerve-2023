package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.utils.Utils;
import frc.robot.utils.math.differential.Integral;

public class IntakeIOSim implements IntakeIO{
    private final FlywheelSim angleMotor;
    private final FlywheelSim spinMotor;

    private final Integral currentAngle = new Integral(0, 0);
    private final PIDController angleFeedback;

    private double angleMotorAppliedVoltage;
    private double angleSetpoint;

    private double spinMotorAppliedVoltage;

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

        angleFeedback = new PIDController(0, 0, 0, 0.02);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        angleMotor.update(0.02);
        spinMotor.update(0.02);

        currentAngle.update(angleMotor.getAngularVelocityRadPerSec());

        inputs.angle = currentAngle.get();
        inputs.angleSetpoint = angleSetpoint;
        inputs.angleMotorAppliedVoltage = angleMotorAppliedVoltage;

        inputs.spinMotorAppliedVoltage = spinMotorAppliedVoltage;
    }

    @Override
    public double getAngle() {
        return currentAngle.get();
    }

    @Override
    public void setAngle(double angle) {
        angleSetpoint = angle;
        angleMotorAppliedVoltage = angleFeedback.calculate(MathUtil.angleModulus(currentAngle.get()), angle);
        angleMotor.setInputVoltage(angleMotorAppliedVoltage);
    }

    @Override
    public void setSpinMotorPower(double power) {
        spinMotorAppliedVoltage = power*IntakeConstants.VOLT_COMPENSATION_SATURATION;
        spinMotor.setInputVoltage(spinMotorAppliedVoltage);
    }
}

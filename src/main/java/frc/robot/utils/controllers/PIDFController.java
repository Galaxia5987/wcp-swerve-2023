package frc.robot.utils.controllers;

import edu.wpi.first.math.controller.PIDController;

public class PIDFController extends PIDController {
    private final double kF;

    public PIDFController(double kp, double ki, double kd, double kF) {
        super(kp, ki, kd);
        this.kF = kF;
    }

    public PIDFController(double kp, double ki, double kd, double kF, double period) {
        super(kp, ki, kd, period);
        this.kF = kF;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        double val = super.calculate(measurement);
        val += Math.signum(val) * kF;
        return val;
    }
}

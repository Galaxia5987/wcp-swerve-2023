package frc.robot.subsystems.hood;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.LoggedSubsystem;
import frc.robot.utils.Utils;
import frc.robot.utils.motors.PIDTalon;
import frc.robot.utils.units.UnitModel;
import frc.robot.utils.valuetuner.WebConstant;

public class Hood extends LoggedSubsystem {
    private static Hood INSTANCE = null;
    private final PIDTalon motor;
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(9);
    private final UnitModel unitModelPosition = new UnitModel(Constants.Hood.TICKS_PER_DEGREE);
    private final WebConstant webKp = WebConstant.of("Hood", "kP", Constants.Hood.Kp);
    private final WebConstant webKi = WebConstant.of("Hood", "kI", Constants.Hood.Ki);
    private final WebConstant webKd = WebConstant.of("Hood", "kD", Constants.Hood.Kd);
    private final WebConstant webKf = WebConstant.of("Hood", "kF", Constants.Hood.Kf);
    private final HoodLogInputs inputs = HoodLogInputs.getInstance();
    private double initialAngle;
    private double setpoint;
    private boolean hasSetInitialValue = false;

    private Hood() {
        super(HoodLogInputs.getInstance());
        motor = new PIDTalon(Ports.Hood.MOTOR);
        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(Ports.Hood.INVERSION);
        motor.configNeutralDeadband(0.01);
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.TALON_TIMEOUT);

        System.out.println("Absolute: " + (encoder.isConnected()));
        motor.setSelectedSensorPosition(0);
        motor.configMotionCruiseVelocity(unitModelPosition.toTicks100ms(Constants.Hood.MAX_VELOCITY));
        motor.configMotionAcceleration(unitModelPosition.toTicks100ms(Constants.Hood.MAX_ACCELERATION));

        motor.enableVoltageCompensation(true);
        motor.configVoltageCompSaturation(10);
    }

    public static Hood getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Hood();
        }
        return INSTANCE;
    }

    public double getAngle() {
        return inputs.angle;
    }

    public void setAngle(double angle) {
        double currentAngle = inputs.angle;
        double error = angle - currentAngle;

        motor.set(ControlMode.MotionMagic, inputs.relativeTicks + unitModelPosition.toTicks(error));
        setpoint = angle;
    }

    public void setPower(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean atSetpoint(double tolerance) {
        return Utils.epsilonEquals(inputs.angle, setpoint, tolerance);
    }

    public boolean atSetpoint() {
        return atSetpoint(Constants.Hood.ALLOWABLE_ERROR);
    }

    public void updatePID() {
        motor.config_kP(0, webKp.get());
        motor.config_kI(0, webKi.get());
        motor.config_kD(0, webKd.get());
        motor.config_kF(0, webKf.get());
    }

    @Override
    public void periodic() {
        if (encoder.isConnected() && !hasSetInitialValue) {
            hasSetInitialValue = true;
            initialAngle = (encoder.getAbsolutePosition() * 360.0 - Constants.Hood.ZERO_POSITION / 2048 * 360.0);
        }
        updatePID();
    }

    @Override
    public void updateInputs() {
        inputs.ticks = encoder.getAbsolutePosition() * 2048;
        inputs.relativeTicks = motor.getSelectedSensorPosition();
        inputs.angle = Math.IEEEremainder(unitModelPosition.toUnits(motor.getSelectedSensorPosition()) + initialAngle, 360.0);
        inputs.setpoint = setpoint;
        inputs.velocity = unitModelPosition.toVelocity(motor.getSelectedSensorVelocity());
    }

    @Override
    public String getSubsystemName() {
        return "Hood";
    }
}

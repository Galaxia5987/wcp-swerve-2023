package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class IntakeConstants {
    public static final double GEAR_RATIO = 35.26;

    public static final double VOLT_COMPENSATION_SATURATION = 12;

    public static final SupplyCurrentLimitConfiguration ANGLE_MOTOR_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 50, 0, 0);

    public static final int SPIN_MOTOR_CURRENT_LIMIT = 40;

    public static final double ANGLE_UP = -0.0610865; //[rad]
    public static final double ANGLE_DOWN = -1.72788; //[rad]

    public static final double ANGLE_MOTOR_kP = 0;
    public static final double ANGLE_MOTOR_kI = 0;
    public static final double ANGLE_MOTOR_kD = 0;
    public static final double ANGLE_MOTOR_kF = 0;

    public static final double ANGLE_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double SPIN_MOTOR_MOMENT_OF_INERTIA = 0;
    public static final double TICKS_PER_RADIAN = 2048/(Math.PI*2)*GEAR_RATIO;

    public static final double MAX_CURRENT = 12;

}

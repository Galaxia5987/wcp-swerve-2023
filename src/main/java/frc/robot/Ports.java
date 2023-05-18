package frc.robot;

public final class Ports {

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 5;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 2;
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 4;
    public static final int REAR_LEFT_MODULE_DRIVE_MOTOR_ID = 7;
    public static final int REAR_LEFT_MODULE_STEER_MOTOR_ID = 8;
    public static final int REAR_RIGHT_MODULE_DRIVE_MOTOR_ID = 1;
    public static final int REAR_RIGHT_MODULE_STEER_MOTOR_ID = 6;

    public static final int FRONT_LEFT_ENCODER_ID = 6;
    public static final int FRONT_RIGHT_ENCODER_ID = 8;
    public static final int REAR_LEFT_ENCODER_ID = 5;
    public static final int REAR_RIGHT_ENCODER_ID = 7;

    public static boolean FRONT_LEFT_DRIVE_INVERTED = true;
    public static boolean FRONT_LEFT_ANGLE_INVERTED = true;
    public static boolean FRONT_RIGHT_DRIVE_INVERTED = true;
    public static boolean FRONT_RIGHT_ANGLE_INVERTED = true;
    public static boolean REAR_LEFT_DRIVE_INVERTED = true;
    public static boolean REAR_LEFT_ANGLE_INVERTED = true;
    public static boolean REAR_RIGHT_DRIVE_INVERTED = true;
    public static boolean REAR_RIGHT_ANGLE_INVERTED = true;

    public static class GripperPorts {
        public static int GRIPPER_MAIN_MOTOR = 0;
        public static int GRIPPER_SPIN_MOTOR_LEFT = 0;
        public static int GRIPPER_SPIN_MOTOR_RIGHT = 0;
    }

    public static class UI {
        public static final int JOYSTICK_TRIGGER = 1;
        public static final int JOYSTICK_TOP_BOTTOM_BUTTON = 2;
        public static final int JOYSTICK_TOP_LEFT_BUTTON = 3;
        public static final int JOYSTICK_TOP_RIGHT_BUTTON = 4;
    }
}


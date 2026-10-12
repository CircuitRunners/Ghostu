package org.firstinspires.ftc.teamcode;

public class Config {
    public static final String FL = "fl", FR = "fr", BL = "bl", BR = "br";
    public static final String OUTTAKE_LEFT = "shootL", OUTTAKE_RIGHT = "shootR";
    public static final String ODO_PARALLEL = "odoPar", ODO_PERPENDICULAR = "odoPerp";
    public static final String AXON_1 = "ax1", AXON_2 = "ax2", AXON_3 = "ax3", AXON_4 = "ax4";
    public static final String SERVO_25_1 = "s1", SERVO_25_2 = "s2", SERVO_25_3 = "s3";
    public static final String IMU = "imu", COLOR = "color";


    public static final boolean REVERSE_FL = true, REVERSE_FR = false;
    public static final boolean REVERSE_BL = true, REVERSE_BR = false;
    public static final boolean REVERSE_OUTTAKE_LEFT = false, REVERSE_OUTTAKE_RIGHT = true;
    public static final boolean REVERSE_AXON_1 = false, REVERSE_AXON_2 = false;
    public static final boolean REVERSE_AXON_3 = false, REVERSE_AXON_4 = false;
    public static final boolean REVERSE_SERVO_25_1 = false, REVERSE_SERVO_25_2 = false;
    public static final boolean REVERSE_SERVO_25_3 = false;


    public static final double SERVO_MIN = 0.0, SERVO_MAX = 1.0;
    public static final double[] HOME     = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] INTAKE   = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] TRANSFER = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] SCORE    = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};


    public static final double DRIVE_DEADBAND = 0.05;
    public static final double MAX_DRIVE_POWER = 1.0;
    public static final double SLOW_DRIVE_POWER = 0.4;
    public static final double DRIVE_RAMP_PER_LOOP = 0.1;


    public static final double OUTTAKE_MAX_POWER = 1.0;
    public static final double OUTTAKE_SLEW_PER_LOOP = 0.05;
    public static final double OUTTAKE_STALL_TIME_MS = 500;
    public static final double OUTTAKE_CURRENT_LIMIT_AMPS = 8.0;
}
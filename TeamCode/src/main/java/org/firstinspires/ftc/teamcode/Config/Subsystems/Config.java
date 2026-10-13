package org.firstinspires.ftc.teamcode.Config.Subsystems;

public class Config {
    public static final String FL = "fl", FR = "fr", BL = "bl", BR = "br";
    public static final String OUTTAKE_LEFT = "shootL", OUTTAKE_RIGHT = "shootR";
    public static final String TRANSFER_LEFT = "transferL", TRANSFER_RIGHT = "transferR";
    public static final String BLOCKER_LEFT = "blockL", BLOCKER_RIGHT = "blockR";
    public static final String ODO_PARALLEL = "odoPar", ODO_PERPENDICULAR = "odoPerp";
    public static final String AXON_1 = "ax1", AXON_2 = "ax2", AXON_3 = "ax3", AXON_4 = "ax4";
    public static final String SERVO_25_1 = "s1", SERVO_25_2 = "s2", SERVO_25_3 = "s3";
    public static final String IMU = "imu", COLOR = "color";


    public static final boolean REVERSE_FL = true, REVERSE_FR = false;
    public static final boolean REVERSE_BL = true, REVERSE_BR = false;
    public static final boolean REVERSE_OUTTAKE_LEFT = false, REVERSE_OUTTAKE_RIGHT = true;
    // Transfer motors are mounted facing opposite ways, so one is reversed:
    // positive power then feeds game pieces toward the shooter on both.
    public static final boolean REVERSE_TRANSFER_LEFT = false, REVERSE_TRANSFER_RIGHT = true;
    public static final boolean REVERSE_BLOCKER_LEFT = false, REVERSE_BLOCKER_RIGHT = false;
    public static final boolean REVERSE_AXON_1 = false, REVERSE_AXON_2 = false;
    public static final boolean REVERSE_AXON_3 = false, REVERSE_AXON_4 = false;
    public static final boolean REVERSE_SERVO_25_1 = false, REVERSE_SERVO_25_2 = false;
    public static final boolean REVERSE_SERVO_25_3 = false;


    public static final double SERVO_MIN = 0.0, SERVO_MAX = 1.0;
    public static final double[] HOME     = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] INTAKE   = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] TRANSFER = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] SCORE    = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};


    // Transfer blockers (gates that hold game pieces back from the shooter)
    public static final double BLOCKER_BLOCK_POS = 0.5;
    public static final double BLOCKER_OPEN_POS = 0.0;   // TODO: tune on robot, open position unknown
    public static final double TRANSFER_POWER = 1.0;


    public static final double DRIVE_DEADBAND = 0.05;
    public static final double MAX_DRIVE_POWER = 1.0;
    public static final double SLOW_DRIVE_POWER = 0.4;
    public static final double DRIVE_RAMP_PER_LOOP = 0.1;


    public static final boolean STRAFE_POSITIVE_RIGHT = true;


    public static final double OUTTAKE_MAX_POWER = 1.0;
    public static final double OUTTAKE_SLEW_PER_LOOP = 0.05;
    public static final double OUTTAKE_STALL_TIME_MS = 500;
    public static final double OUTTAKE_CURRENT_LIMIT_AMPS = 8.0;



    public static final double AUTO_LENGTH_SEC = 30.0;
    public static final double AUTO_PARK_WINDOW_SEC = 10.0;

    public static final boolean AUTO_WAIT_FOR_PARK_WINDOW = false;

    public static final double AUTO_PARK_X_IN = 24, AUTO_PARK_Y_IN = 36, AUTO_PARK_HEADING_DEG = 0;



    public static final boolean FENCE_ENABLED_DEFAULT = true;
    public static final double FIELD_MIN_IN = 0.0, FIELD_MAX_IN = 144.0;
    public static final double ROBOT_RADIUS_IN = 9.0;
    public static final double FENCE_SLOW_DIST_IN = 12.0;


    public static final double TELEOP_START_X_IN = 72, TELEOP_START_Y_IN = 9, TELEOP_START_HEADING_DEG = 90;


    public static final double[][] KEEP_OUT_ZONES = {
    };



    public static final boolean OBSTACLE_AVOID_ENABLED_DEFAULT = true;

    public static final double ODO_TICKS_PER_INCH = 505.0;
    public static final double OBSTACLE_MIN_CMD = 0.35;
    public static final double OBSTACLE_MIN_SPEED_IN_S = 2.0;
    public static final double OBSTACLE_STALL_MS = 300;
    public static final double OBSTACLE_RELEASE_CMD = 0.10;
}

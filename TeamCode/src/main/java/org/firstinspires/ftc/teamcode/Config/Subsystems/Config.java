package org.firstinspires.ftc.teamcode.Config.Subsystems;

public class Config {
    public static final String FL = "fl", FR = "fr", BL = "bl", BR = "br";
    public static final String OUTTAKE_LEFT = "shootL", OUTTAKE_RIGHT = "shootR";
    public static final String TRANSFER_LEFT = "transferL", TRANSFER_RIGHT = "transferR";
    public static final String BLOCKER_LEFT = "blockL", BLOCKER_RIGHT = "blockR";
    public static final String AXON_1 = "ax1", AXON_2 = "ax2", AXON_3 = "ax3", AXON_4 = "ax4";
    public static final String SERVO_25_1 = "s1", SERVO_25_2 = "s2", SERVO_25_3 = "s3";
    public static final String IMU = "imu", COLOR = "color";

    public static final boolean REVERSE_FL = true, REVERSE_FR = false;
    public static final boolean REVERSE_BL = true, REVERSE_BR = false;
    public static final boolean REVERSE_OUTTAKE_LEFT = false, REVERSE_OUTTAKE_RIGHT = true;
    public static final boolean REVERSE_TRANSFER_LEFT = false, REVERSE_TRANSFER_RIGHT = true;
    public static final boolean REVERSE_BLOCKER_LEFT = false, REVERSE_BLOCKER_RIGHT = false;
    public static final boolean REVERSE_AXON_1 = false, REVERSE_AXON_2 = false;
    public static final boolean REVERSE_AXON_3 = false, REVERSE_AXON_4 = false;
    public static final boolean REVERSE_SERVO_25_1 = false, REVERSE_SERVO_25_2 = false;
    public static final boolean REVERSE_SERVO_25_3 = false;

    public static final double[] HOME     = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] INTAKE   = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] TRANSFER = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    public static final double[] SCORE    = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    public static final double BLOCKER_BLOCK_POS = 0.5;
    public static final double BLOCKER_OPEN_POS = 0.0;

    public static final double SLOW_DRIVE_POWER = 0.4;

    public static final double AUTO_PARK_X_IN = 24, AUTO_PARK_Y_IN = 36, AUTO_PARK_HEADING_DEG = 0;
}

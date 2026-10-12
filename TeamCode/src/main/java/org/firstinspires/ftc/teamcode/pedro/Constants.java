package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    // Drivetrain: values mirror your Config class (FL/BL reversed, FR/BR forward).
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // Localizer: two odometry pods + Control Hub IMU.
    // Run the "Two Wheel Tuner" in AutoTune (http://192.168.43.1:10158), open the Java tab,
    // and paste the generated values in here. It produces: encoder names, IMU name/orientation,
    // forward/strafe ticks-to-inches, x/y pod directions, and x/y pod offsets.
    //
    // Starting estimate for ticks-to-inches from your Config (pi * 32mm / 2000 ticks):
    //   about 0.0019789 inches per tick (let the tuner refine it).
    // Your "odoParallel" / "odoPerpendicular" names are the forward and strafe encoder names,
    // and they must be the hardware-map names of the MOTOR PORTS the encoders plug into.
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(
            c -> {
                // PASTE AutoTune OUTPUT HERE
            }
    );

    // Foresight (path-following algorithm): paste the values from the Foresight tuner here.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // PASTE Foresight tuner OUTPUT HERE
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
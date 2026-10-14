package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Robot;

public class Drive {
    private final Robot robot;

    public Drive(Robot robot) {
        this.robot = robot;
    }

    public void drive(double strafe, double forward, double turn, boolean fieldCentric, double scale) {
        if (fieldCentric) {
            double heading = -robot.getHeadingRadians();
            double newStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);
            double newForward = strafe * Math.sin(heading) + forward * Math.cos(heading);
            strafe = newStrafe;
            forward = newForward;
        }

        double denominator = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        robot.frontLeft.setPower((forward + strafe + turn) / denominator * scale);
        robot.frontRight.setPower((forward - strafe - turn) / denominator * scale);
        robot.backLeft.setPower((forward - strafe + turn) / denominator * scale);
        robot.backRight.setPower((forward + strafe - turn) / denominator * scale);
    }

    public void stop() {
        drive(0, 0, 0, false, 0);
    }
}

package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Robot;

public class Shooter {
    private final Robot robot;

    public Shooter(Robot robot) {
        this.robot = robot;
    }

    public void setPower(double power) {
        robot.outtakeLeft.setPower(power);
        robot.outtakeRight.setPower(power);
    }

    public void stop() {
        setPower(0);
    }
}

package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Robot;

public class Intake {
    private final Robot robot;

    public Intake(Robot robot) {
        this.robot = robot;
    }

    public void setIntakePosition() {
        robot.setServoPositions(Config.INTAKE);
    }

    public void setHomePosition() {
        robot.setServoPositions(Config.HOME);
    }
}

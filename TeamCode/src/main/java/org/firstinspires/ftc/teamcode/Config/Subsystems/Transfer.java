package org.firstinspires.ftc.teamcode.Config.Subsystems;

import org.firstinspires.ftc.teamcode.Config.Robot;

public class Transfer {
    private final Robot robot;

    public Transfer(Robot robot) {
        this.robot = robot;
    }

    public void setTransferPosition() {
        robot.setServoPositions(Config.TRANSFER);
    }

    public void setScorePosition() {
        robot.setServoPositions(Config.SCORE);
    }

    public void block() {
        robot.setBlockerPositions(Config.BLOCKER_BLOCK_POS);
    }

    public void open() {
        robot.setBlockerPositions(Config.BLOCKER_OPEN_POS);
    }

    public void setPower(double power) {
        robot.setTransferPower(power);
    }

    public void feed() {
        open();
        setPower(1.0);
    }

    public void unjam() {
        block();
        setPower(-1.0);
    }

    public void hold() {
        block();
        setPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Config.Robot;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;

public class Transfer {
    private final Robot robot;

    public Transfer(Robot robot) {
        this.robot = robot;
    }

    // Arm/servo presets (unchanged)
    public void setTransferPosition() {
        robot.setServoPositions(Config.TRANSFER);
    }

    public void setScorePosition() {
        robot.setServoPositions(Config.SCORE);
    }

    // Blocker gates
    public void block() {
        robot.setBlockerPositions(Config.BLOCKER_BLOCK_POS);
    }

    public void open() {
        robot.setBlockerPositions(Config.BLOCKER_OPEN_POS);
    }

    // Transfer motors (positive = toward shooter; the opposite-direction mounting
    // is handled by Config.REVERSE_TRANSFER_*)
    public void setPower(double power) {
        robot.setTransferPower(power * Config.TRANSFER_POWER);
    }

    /** Gates open, motors feeding forward. */
    public void feed() {
        open();
        setPower(1.0);
    }

    /** Gates closed, motors backward to clear a jam. */
    public void unjam() {
        block();
        setPower(-1.0);
    }

    /** Gates closed, motors off. */
    public void hold() {
        block();
        setPower(0);
    }
}

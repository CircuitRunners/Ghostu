package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Robot;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BIOBUZZ Hardware Diagnostics", group = "Diagnostics")
public class Diagnostics extends LinearOpMode {
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        int servoIndex = 0;
        double[] positions = Config.HOME.clone();
        double blockerPos = Config.BLOCKER_BLOCK_POS;
        boolean lastLeft = false, lastRight = false, lastUp = false, lastDown = false;
        boolean lastG2Up = false, lastG2Down = false;

        telemetry.addLine("DIAGNOSTICS READY");
        telemetry.addLine("P1: dpad L/R = pick servo, dpad U/D = nudge");
        telemetry.addLine("P2: left stick = drive test, triggers = shooter");
        telemetry.addLine("P2: dpad U/D = nudge blockers, RB = transfer fwd, LB = transfer back");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !lastLeft) servoIndex = Math.max(0, servoIndex - 1);
            if (gamepad1.dpad_right && !lastRight) servoIndex = Math.min(6, servoIndex + 1);
            if (gamepad1.dpad_up && !lastUp) positions[servoIndex] = Range.clip(positions[servoIndex] + 0.02, 0, 1);
            if (gamepad1.dpad_down && !lastDown) positions[servoIndex] = Range.clip(positions[servoIndex] - 0.02, 0, 1);
            lastLeft = gamepad1.dpad_left;
            lastRight = gamepad1.dpad_right;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            robot.servos[servoIndex].setPosition(positions[servoIndex]);

            if (gamepad2.dpad_up && !lastG2Up) blockerPos = Range.clip(blockerPos + 0.02, 0, 1);
            if (gamepad2.dpad_down && !lastG2Down) blockerPos = Range.clip(blockerPos - 0.02, 0, 1);
            lastG2Up = gamepad2.dpad_up;
            lastG2Down = gamepad2.dpad_down;
            robot.setBlockerPositions(blockerPos);

            double transferPower = 0;
            if (gamepad2.right_bumper) transferPower = 0.5;
            else if (gamepad2.left_bumper) transferPower = -0.5;
            robot.setTransferPower(transferPower);

            double drivePower = -gamepad2.left_stick_y * 0.20;
            robot.frontLeft.setPower(drivePower);
            robot.frontRight.setPower(drivePower);
            robot.backLeft.setPower(drivePower);
            robot.backRight.setPower(drivePower);

            double shooterPower = gamepad2.right_trigger - gamepad2.left_trigger;
            robot.outtakeLeft.setPower(shooterPower);
            robot.outtakeRight.setPower(shooterPower);

            telemetry.addData("Servo Index", servoIndex + 1);
            telemetry.addData("Position", "%.3f", positions[servoIndex]);
            telemetry.addData("Blocker Pos", "%.3f", blockerPos);
            telemetry.addData("Transfer Power", "%.2f", transferPower);
            telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(robot.getHeadingRadians()));
            telemetry.update();
        }

        robot.stopAll();
    }
}

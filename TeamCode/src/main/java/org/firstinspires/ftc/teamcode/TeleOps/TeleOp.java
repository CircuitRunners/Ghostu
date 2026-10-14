package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Robot;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Transfer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    private final Robot robot = new Robot();
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new Drive(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        shooter = new Shooter(robot);

        telemetry.addLine("BIOBUZZ READY");
        telemetry.addLine("P1: START = reset heading | RB = robot-centric | LB = slow");
        telemetry.addLine("P2: A/B/X/Y = presets | hold RB = feed | hold LB = unjam");
        telemetry.addLine("P2: dpad up/down = shooter on/off | triggers = manual shooter");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.resetHeading();
    }

    @Override
    public void loop() {
        if (gamepad1.start) robot.resetHeading();

        double strafe = -gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        boolean robotCentric = gamepad1.right_bumper;
        double scale = gamepad1.left_bumper ? Config.SLOW_DRIVE_POWER : 1.0;

        drive.drive(strafe, forward, turn, !robotCentric, scale);

        if (gamepad2.a) intake.setHomePosition();
        if (gamepad2.b) intake.setIntakePosition();
        if (gamepad2.x) transfer.setTransferPosition();
        if (gamepad2.y) transfer.setScorePosition();

        if (gamepad2.right_bumper) {
            transfer.feed();
        } else if (gamepad2.left_bumper) {
            transfer.unjam();
        } else {
            transfer.hold();
        }

        if (gamepad2.dpad_up) shooter.setPower(1.0);
        if (gamepad2.dpad_down) shooter.setPower(0.0);

        double manualShooter = gamepad2.right_trigger - gamepad2.left_trigger;
        if (Math.abs(manualShooter) > 0.05) shooter.setPower(manualShooter);

        telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(robot.getHeadingRadians()));
        telemetry.addData("Drive Mode", robotCentric ? "Robot" : "Field");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}

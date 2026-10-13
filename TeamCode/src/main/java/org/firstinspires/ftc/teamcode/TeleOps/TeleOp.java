package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.FieldFence;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.ObstacleAvoidance;
import org.firstinspires.ftc.teamcode.Config.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BIOBUZZ TeleOp", group = "Competition")
public class TeleOp extends OpMode {
    public enum Preset { HOME, INTAKE, TRANSFER, SCORE }

    private final Robot robot = new Robot();
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Localizer localizer;
    private final FieldFence fence = new FieldFence();
    private ObstacleAvoidance avoidance;

    private Preset currentPreset = Preset.HOME;
    private String transferState = "HOLD";
    private boolean lastStart, lastA, lastB, lastX, lastY, lastDpadUp, lastDpadDown;
    private boolean lastBack1, lastX1, lastY1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new Drive(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        shooter = new Shooter(robot);
        localizer = new Localizer(hardwareMap);
        avoidance = new ObstacleAvoidance(robot);
        reseedPose();

        telemetry.addLine("BIOBUZZ READY");
        telemetry.addLine("P1: BACK = fence on/off | X = obstacle avoid on/off");
        telemetry.addLine("P1: Y = re-seed pose at start spot | hold LT = override both");
        telemetry.addLine("P2: hold RB = feed (gates open) | hold LB = unjam (gates closed)");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.resetHeading();
        intake.setHomePosition();
        shooter.setPower(0);
        transfer.hold();
    }

    private void reseedPose() {
        localizer.resetPose(Config.TELEOP_START_X_IN, Config.TELEOP_START_Y_IN,
                Math.toRadians(Config.TELEOP_START_HEADING_DEG));
    }

    @Override
    public void loop() {
        robot.clearBulkCache();
        localizer.update();
        avoidance.update();

        if (gamepad1.start && !lastStart) robot.resetHeading();
        lastStart = gamepad1.start;


        if (gamepad1.back && !lastBack1) fence.toggle();
        if (gamepad1.x && !lastX1) avoidance.toggle();
        if (gamepad1.y && !lastY1) reseedPose();
        lastBack1 = gamepad1.back;
        lastX1 = gamepad1.x;
        lastY1 = gamepad1.y;

        double strafe = -gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        boolean robotCentric = gamepad1.right_bumper;
        double scale = gamepad1.left_bumper ? Config.SLOW_DRIVE_POWER / Config.MAX_DRIVE_POWER : 1.0;
        boolean override = gamepad1.left_trigger > 0.5;


        double[] cmd = drive.toRobotFrame(strafe, forward, robotCentric);


        Pose pose = localizer.getPose();
        if (!override) {
            cmd = fence.limitRobotFrame(cmd[0], cmd[1], pose.x(), pose.y(), pose.heading());
            cmd = avoidance.limit(cmd[0], cmd[1], scale);
        } else {
            avoidance.reset();
        }


        drive.drive(cmd[0], cmd[1], turn, true, scale);

        if (gamepad2.a && !lastA) {
            intake.setHomePosition();
            currentPreset = Preset.HOME;
        }
        if (gamepad2.b && !lastB) {
            intake.setIntakePosition();
            currentPreset = Preset.INTAKE;
        }
        if (gamepad2.x && !lastX) {
            transfer.setTransferPosition();
            currentPreset = Preset.TRANSFER;
        }
        if (gamepad2.y && !lastY) {
            transfer.setScorePosition();
            currentPreset = Preset.SCORE;
        }
        lastA = gamepad2.a;
        lastB = gamepad2.b;
        lastX = gamepad2.x;
        lastY = gamepad2.y;

        // Transfer: hold RB to open gates and feed, hold LB to run backward with gates closed
        if (gamepad2.right_bumper) {
            transfer.feed();
            transferState = "FEED";
        } else if (gamepad2.left_bumper) {
            transfer.unjam();
            transferState = "UNJAM";
        } else {
            transfer.hold();
            transferState = "HOLD";
        }

        if (gamepad2.dpad_up && !lastDpadUp) shooter.setPower(1.0);
        if (gamepad2.dpad_down && !lastDpadDown) shooter.setPower(0.0);
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        double manualShooter = gamepad2.right_trigger - gamepad2.left_trigger;
        if (Math.abs(manualShooter) > 0.05) shooter.setPower(manualShooter);
        shooter.update();

        telemetry.addData("Heading Deg", "%.1f", Math.toDegrees(robot.getHeadingRadians()));
        telemetry.addData("Drive Mode", robotCentric ? "Robot" : "Field");
        telemetry.addData("Preset", currentPreset);
        telemetry.addData("Transfer", transferState);
        telemetry.addData("Shooter Power", "%.2f / %.2f", shooter.getCommandedPower(), shooter.getTargetPower());
        telemetry.addData("Shooter Current", "%.2f A", shooter.getCurrentAmps());
        telemetry.addData("Pose X/Y/H", "%.1f / %.1f / %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Fence", override ? "OVERRIDE" : (fence.isEnabled() ? (fence.isLimiting() ? "ON - LIMITING" : "ON") : "OFF"));
        telemetry.addData("Obstacle Guard", override ? "OVERRIDE"
                : (avoidance.isEnabled() ? (avoidance.isLimiting() ? "ON - BLOCKED" : "ON") : "OFF"));
        telemetry.addData("Blocked Fwd/Strafe", "%d / %d", avoidance.forwardBlockedDir(), avoidance.strafeBlockedDir());
        telemetry.addData("Odo Speed Fwd/Strafe (in/s)", "%.1f / %.1f", avoidance.forwardSpeedInS(), avoidance.strafeSpeedInS());
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAll();
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "BIOBUZZ Auto - Pedro 3", group = "Competition")
public class Auto extends LinearOpMode {
    private final Robot robot = new Robot();
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose intakePose = new Pose(0, 24, Math.toRadians(0));
    private final Pose scorePose = new Pose(0, 36, Math.toRadians(0));

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        drive = new Drive(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        shooter = new Shooter(robot);

        // Load Pedro 3 Constants and initialize Follower
        Constants.setConstants();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize Limelight 3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        intake.setHomePosition();
        telemetry.addLine("AUTO READY (PEDRO 3)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        runRoutine();

        limelight.stop();
        robot.stopAll();
    }

    private void runRoutine() {
        // Build Pedro 3 PathChain using BezierLine paths
        PathChain driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(intakePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), intakePose.getHeading())
                .addPath(new BezierLine(new Point(intakePose), new Point(scorePose)))
                .setLinearHeadingInterpolation(intakePose.getHeading(), scorePose.getHeading())
                .build();

        intake.setIntakePosition();
        holdMechanism(250);
        shooter.setPower(0);

        // Follow path chain
        follower.followPath(driveToScore, true);
        while (opModeIsActive() && follower.isBusy()) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();

            Pose currentPose = follower.getPose();
            telemetry.addData("X / Y", "%.2f / %.2f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }

        // Align with target using Limelight 3A
        alignWithLimelight(0.0, 0.0, 2000);

        // Execute scoring sequence
        transfer.setTransferPosition();
        holdMechanism(250);
        transfer.setScorePosition();
        shooter.setPower(1.0);
        holdMechanism(650);
        shooter.setPower(0);
        holdMechanism(200);
        intake.setHomePosition();
        holdMechanism(200);
    }

    private boolean alignWithLimelight(double targetTx, double targetTy, long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double KpAim = 0.04;
        double KpDistance = 0.06;

        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                double headingError = targetTx - tx;
                double distanceError = targetTy - ty;

                if (Math.abs(headingError) < 1.5 && Math.abs(distanceError) < 1.5) {
                    drive.stop();
                    return true;
                }

                double turn = Range.clip(headingError * KpAim, -Config.AUTO_MAX_TURN_POWER, Config.AUTO_MAX_TURN_POWER);
                double forward = Range.clip(distanceError * KpDistance, -Config.AUTO_MAX_TRANSLATION_POWER, Config.AUTO_MAX_TRANSLATION_POWER);

                drive.drive(0, forward, -turn, false, 1.0);

                telemetry.addData("Limelight", "Tracking target");
                telemetry.addData("tx / ty", "%.2f / %.2f", tx, ty);
            } else {
                drive.stop();
                telemetry.addData("Limelight", "Searching for target...");
            }
            telemetry.update();
        }

        drive.stop();
        return false;
    }

    private void holdMechanism(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();
            idle();
        }
    }
}
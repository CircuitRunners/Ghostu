package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Robot;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;

@Autonomous(name = "BIOBUZZ Auto - Pedro 3", group = "Competition")
public class Auto extends LinearOpMode {
    private final Robot robot = new Robot();
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Follower follower;
    private Limelight3A limelight;


    private static final double AUTO_MAX_TURN_POWER = 0.55;
    private static final double AUTO_MAX_TRANSLATION_POWER = 0.70;


    private static final double PARK_START_SEC = Config.AUTO_LENGTH_SEC - Config.AUTO_PARK_WINDOW_SEC;

    private final ElapsedTime autoTimer = new ElapsedTime();


    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(0, 0, 0);
    private final Pose intakePose = p.of(0, 24, 0);
    private final Pose scorePose = p.of(0, 36, 0);
    private final Pose parkPose = p.of(Config.AUTO_PARK_X_IN, Config.AUTO_PARK_Y_IN, Config.AUTO_PARK_HEADING_DEG);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        drive = new Drive(robot);
        intake = new Intake(robot);
        transfer = new Transfer(robot);
        shooter = new Shooter(robot);


        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        intake.setHomePosition();
        transfer.hold();
        telemetry.addLine("AUTO READY (PEDRO 3)");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        autoTimer.reset();

        runRoutine();
        park();

        limelight.stop();
        transfer.hold();
        robot.stopAll();
    }


    private boolean parkWindowOpen() {
        return autoTimer.seconds() >= PARK_START_SEC;
    }

    private Path driveToScore() {
        return Paths.path(
                Paths.line(startPose, intakePose).linear(startPose, intakePose),
                Paths.line(intakePose, scorePose).linear(intakePose, scorePose)
        );
    }

    private void runRoutine() {
        intake.setIntakePosition();
        holdMechanism(250);
        shooter.setPower(0);
        if (parkWindowOpen()) return;


        follower.follow(driveToScore());
        while (opModeIsActive() && follower.isBusy() && !parkWindowOpen()) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();

            Pose currentPose = follower.pose();
            telemetry.addData("X / Y", "%.2f / %.2f", currentPose.x(), currentPose.y());
            telemetry.addData("Heading", "%.1f", Math.toDegrees(currentPose.heading()));
            telemetry.addData("Auto time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }

        follower.stop();
        if (parkWindowOpen()) return;


        alignWithLimelight(0.0, 0.0, 2000);
        if (parkWindowOpen()) return;


        transfer.setTransferPosition();
        holdMechanism(250);
        if (parkWindowOpen()) return;
        transfer.setScorePosition();

        // Spin up with gates closed, then open gates and run the transfer motors to feed
        shooter.setPower(1.0);
        transfer.hold();
        holdMechanism(400);
        if (parkWindowOpen()) {
            transfer.hold();
            return;
        }
        transfer.feed();
        holdMechanism(650);
        transfer.hold();
        shooter.setPower(0);
        holdMechanism(200);
        if (parkWindowOpen()) return;
        intake.setHomePosition();
        holdMechanism(200);
    }


    private void park() {

        while (Config.AUTO_WAIT_FOR_PARK_WINDOW && opModeIsActive() && !parkWindowOpen()) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();
            idle();
        }
        if (!opModeIsActive()) return;


        shooter.setPower(0);
        transfer.hold();
        intake.setHomePosition();
        drive.stop();

        Pose current = follower.pose();
        follower.follow(Paths.path(
                Paths.line(current, parkPose).linear(current, parkPose)
        ));

        while (opModeIsActive() && follower.isBusy()) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();

            telemetry.addData("State", "PARKING");
            telemetry.addData("Auto time", "%.1f", autoTimer.seconds());
            telemetry.update();
        }

        follower.stop();
        shooter.stop();
        telemetry.addData("State", "PARKED");
        telemetry.update();
    }

    private boolean alignWithLimelight(double targetTx, double targetTy, long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double KpAim = 0.04;
        double KpDistance = 0.06;

        while (opModeIsActive() && timer.milliseconds() < timeoutMs && !parkWindowOpen()) {
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

                double turn = Range.clip(headingError * KpAim, -AUTO_MAX_TURN_POWER, AUTO_MAX_TURN_POWER);
                double forward = Range.clip(distanceError * KpDistance, -AUTO_MAX_TRANSLATION_POWER, AUTO_MAX_TRANSLATION_POWER);

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
        while (opModeIsActive() && timer.milliseconds() < milliseconds && !parkWindowOpen()) {
            robot.clearBulkCache();
            follower.update();
            shooter.update();
            idle();
        }
    }
}

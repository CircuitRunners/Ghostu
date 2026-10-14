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
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Transfer;
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

        runRoutine();
        park();

        limelight.stop();
        transfer.hold();
        robot.stopAll();
    }

    private void runRoutine() {
        intake.setIntakePosition();
        pause(250);

        follower.follow(Paths.path(
                Paths.line(startPose, intakePose).linear(startPose, intakePose),
                Paths.line(intakePose, scorePose).linear(intakePose, scorePose)
        ));
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.stop();

        alignWithLimelight(2000);

        transfer.setTransferPosition();
        pause(250);
        transfer.setScorePosition();

        shooter.setPower(1.0);
        transfer.hold();
        pause(400);
        transfer.feed();
        pause(650);
        transfer.hold();
        shooter.setPower(0);
        pause(200);

        intake.setHomePosition();
        pause(200);
    }

    private void park() {
        Pose current = follower.pose();
        follower.follow(Paths.path(
                Paths.line(current, parkPose).linear(current, parkPose)
        ));
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
        follower.stop();

        telemetry.addData("State", "PARKED");
        telemetry.update();
    }

    private void alignWithLimelight(long timeoutMs) {
        double kAim = 0.04;
        double kDistance = 0.06;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double headingError = -result.getTx();
                double distanceError = -result.getTy();

                if (Math.abs(headingError) < 1.5 && Math.abs(distanceError) < 1.5) break;

                double turn = Range.clip(headingError * kAim, -0.55, 0.55);
                double forward = Range.clip(distanceError * kDistance, -0.7, 0.7);
                drive.drive(0, forward, -turn, false, 1.0);
            } else {
                drive.stop();
            }
        }
        drive.stop();
    }

    private void pause(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            follower.update();
        }
    }
}

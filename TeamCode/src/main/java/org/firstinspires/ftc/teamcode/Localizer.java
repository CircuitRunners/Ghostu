package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Localizer {
    private final Follower follower;

    public Localizer(HardwareMap hardwareMap) {
        Constants.setConstants();
        this.follower = new Follower(hardwareMap);
    }

    public void update() {
        follower.update();
    }

    public void resetPose(double x, double y, double headingRad) {
        follower.setStartingPose(new Pose(x, y, headingRad));
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Follower getFollower() {
        return follower;
    }
}
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Localizer {
    private final Follower follower;
    private final PoseFactory radians = PoseFactory.radians();

    public Localizer(HardwareMap hardwareMap) {

        this.follower = Constants.create(hardwareMap);
    }


    public Localizer(Follower follower) {
        this.follower = follower;
    }

    public void update() {
        follower.update();
    }


    public void setStartingPose(double x, double y, double headingRad) {
        follower.setPose(radians.of(x, y, headingRad));
    }


    public void resetPose(double x, double y, double headingRad) {
        follower.setPose(radians.of(x, y, headingRad));
    }

    public Pose getPose() {
        return follower.pose();
    }

    public Follower getFollower() {
        return follower;
    }
}
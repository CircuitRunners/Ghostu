package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;


public class ObstacleAvoidance {
    private final Robot robot;
    private boolean enabled = Config.OBSTACLE_AVOID_ENABLED_DEFAULT;

    private final AxisGuard forwardGuard = new AxisGuard();
    private final AxisGuard strafeGuard = new AxisGuard();

    private double forwardSpeedInS, strafeSpeedInS;

    public ObstacleAvoidance(Robot robot) {
        this.robot = robot;
    }

    public boolean isEnabled() { return enabled; }
    public void toggle() { enabled = !enabled; reset(); }
    public boolean isLimiting() { return forwardGuard.blockedDir != 0 || strafeGuard.blockedDir != 0; }


    public int forwardBlockedDir() { return forwardGuard.blockedDir; }
    public int strafeBlockedDir() { return strafeGuard.blockedDir; }
    public double forwardSpeedInS() { return forwardSpeedInS; }
    public double strafeSpeedInS() { return strafeSpeedInS; }


    public void reset() {
        forwardGuard.clear();
        strafeGuard.clear();
    }


    public void update() {
        forwardSpeedInS = Math.abs(robot.odoParallel.getVelocity()) / Config.ODO_TICKS_PER_INCH;
        strafeSpeedInS = Math.abs(robot.odoPerpendicular.getVelocity()) / Config.ODO_TICKS_PER_INCH;
    }


    public double[] limit(double strafe, double forward, double scale) {
        if (!enabled) {
            reset();
            return new double[]{strafe, forward};
        }
        double f = forwardGuard.apply(forward, scale, forwardSpeedInS);
        double s = strafeGuard.apply(strafe, scale, strafeSpeedInS);
        return new double[]{s, f};
    }


    private static class AxisGuard {
        int blockedDir = 0;
        private final ElapsedTime timer = new ElapsedTime();
        private int countingDir = 0;

        void clear() {
            blockedDir = 0;
            countingDir = 0;
        }

        double apply(double cmd, double scale, double speedInS) {
            double effective = Math.abs(cmd) * scale;
            int dir = (int) Math.signum(cmd);


            if (blockedDir != 0 && (dir != blockedDir || effective < Config.OBSTACLE_RELEASE_CMD)) {
                blockedDir = 0;
            }
            if (blockedDir != 0) return 0.0;

            boolean pushingHard = effective >= Config.OBSTACLE_MIN_CMD;
            boolean notMoving = speedInS < Config.OBSTACLE_MIN_SPEED_IN_S;

            if (pushingHard && notMoving) {
                if (countingDir != dir) {
                    countingDir = dir;
                    timer.reset();
                } else if (timer.milliseconds() >= Config.OBSTACLE_STALL_MS) {
                    blockedDir = dir;
                    countingDir = 0;
                    return 0.0;
                }
            } else {
                countingDir = 0;
            }
            return cmd;
        }
    }
}

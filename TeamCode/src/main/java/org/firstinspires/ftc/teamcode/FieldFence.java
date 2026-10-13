package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;


public class FieldFence {
    private boolean enabled = Config.FENCE_ENABLED_DEFAULT;
    private boolean limiting;

    public boolean isEnabled() { return enabled; }
    public void setEnabled(boolean enabled) { this.enabled = enabled; }
    public void toggle() { enabled = !enabled; }

    public boolean isLimiting() { return limiting; }


    public double[] limitRobotFrame(double strafe, double forward, double x, double y, double heading) {
        limiting = false;
        double[] original = {strafe, forward};
        if (!enabled || Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) return original;

        double sR = Config.STRAFE_POSITIVE_RIGHT ? 1.0 : -1.0;
        double right = strafe * sR;
        double cos = Math.cos(heading), sin = Math.sin(heading);


        double vx = forward * cos + right * sin;
        double vy = forward * sin - right * cos;

        double[] v = limitFieldFrame(x, y, vx, vy);
        if (Math.abs(v[0] - vx) < 1e-6 && Math.abs(v[1] - vy) < 1e-6) return original;
        limiting = true;


        double f = v[0] * cos + v[1] * sin;
        double r = v[0] * sin - v[1] * cos;
        return new double[]{r * sR, f};
    }

    private double[] limitFieldFrame(double x, double y, double vx, double vy) {
        double m = Config.ROBOT_RADIUS_IN;


        if (vx < 0) vx *= factor(x - (Config.FIELD_MIN_IN + m));
        if (vx > 0) vx *= factor((Config.FIELD_MAX_IN - m) - x);
        if (vy < 0) vy *= factor(y - (Config.FIELD_MIN_IN + m));
        if (vy > 0) vy *= factor((Config.FIELD_MAX_IN - m) - y);


        for (double[] z : Config.KEEP_OUT_ZONES) {
            double x0 = z[0] - m, y0 = z[1] - m, x1 = z[2] + m, y1 = z[3] + m;
            boolean inX = x > x0 && x < x1;
            boolean inY = y > y0 && y < y1;

            if (inX && inY) {

                double dl = x - x0, dr = x1 - x, db = y - y0, dt = y1 - y;
                double min = Math.min(Math.min(dl, dr), Math.min(db, dt));
                if (min == dl) vx = Math.min(vx, 0);
                else if (min == dr) vx = Math.max(vx, 0);
                else if (min == db) vy = Math.min(vy, 0);
                else vy = Math.max(vy, 0);
            } else if (inY) {
                if (x <= x0 && vx > 0) vx *= factor(x0 - x);
                else if (x >= x1 && vx < 0) vx *= factor(x - x1);
            } else if (inX) {
                if (y <= y0 && vy > 0) vy *= factor(y0 - y);
                else if (y >= y1 && vy < 0) vy *= factor(y - y1);
            }
        }
        return new double[]{vx, vy};
    }

    private double factor(double distanceToBoundary) {
        return Range.clip(distanceToBoundary / Config.FENCE_SLOW_DIST_IN, 0.0, 1.0);
    }
}

package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config.Subsystems.Config;

public class Robot {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor outtakeLeft, outtakeRight;
    public DcMotor transferLeft, transferRight;

    public Servo axon1, axon2, axon3, axon4;
    public Servo servo25_1, servo25_2, servo25_3;
    public Servo blockerLeft, blockerRight;

    public IMU imu;
    public ColorSensor color;

    public final Servo[] servos = new Servo[7];


    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, Config.FL);
        frontRight = hardwareMap.get(DcMotor.class, Config.FR);
        backLeft = hardwareMap.get(DcMotor.class, Config.BL);
        backRight = hardwareMap.get(DcMotor.class, Config.BR);

        outtakeLeft = hardwareMap.get(DcMotor.class, Config.OUTTAKE_LEFT);
        outtakeRight = hardwareMap.get(DcMotor.class, Config.OUTTAKE_RIGHT);

        transferLeft = hardwareMap.get(DcMotor.class, Config.TRANSFER_LEFT);
        transferRight = hardwareMap.get(DcMotor.class, Config.TRANSFER_RIGHT);

        axon1 = hardwareMap.get(Servo.class, Config.AXON_1);
        axon2 = hardwareMap.get(Servo.class, Config.AXON_2);
        axon3 = hardwareMap.get(Servo.class, Config.AXON_3);
        axon4 = hardwareMap.get(Servo.class, Config.AXON_4);
        servo25_1 = hardwareMap.get(Servo.class, Config.SERVO_25_1);
        servo25_2 = hardwareMap.get(Servo.class, Config.SERVO_25_2);
        servo25_3 = hardwareMap.get(Servo.class, Config.SERVO_25_3);

        blockerLeft = hardwareMap.get(Servo.class, Config.BLOCKER_LEFT);
        blockerRight = hardwareMap.get(Servo.class, Config.BLOCKER_RIGHT);

        imu = hardwareMap.get(IMU.class, Config.IMU);
        color = hardwareMap.get(ColorSensor.class, Config.COLOR);

        frontLeft.setDirection(Config.REVERSE_FL ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frontRight.setDirection(Config.REVERSE_FR ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backLeft.setDirection(Config.REVERSE_BL ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        backRight.setDirection(Config.REVERSE_BR ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        outtakeLeft.setDirection(Config.REVERSE_OUTTAKE_LEFT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        outtakeRight.setDirection(Config.REVERSE_OUTTAKE_RIGHT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        transferLeft.setDirection(Config.REVERSE_TRANSFER_LEFT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        transferRight.setDirection(Config.REVERSE_TRANSFER_RIGHT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        configureMotor(frontLeft);
        configureMotor(frontRight);
        configureMotor(backLeft);
        configureMotor(backRight);
        configureMotor(outtakeLeft);
        configureMotor(outtakeRight);
        configureMotor(transferLeft);
        configureMotor(transferRight);

        axon1.setDirection(Config.REVERSE_AXON_1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        axon2.setDirection(Config.REVERSE_AXON_2 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        axon3.setDirection(Config.REVERSE_AXON_3 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        axon4.setDirection(Config.REVERSE_AXON_4 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo25_1.setDirection(Config.REVERSE_SERVO_25_1 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo25_2.setDirection(Config.REVERSE_SERVO_25_2 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servo25_3.setDirection(Config.REVERSE_SERVO_25_3 ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        blockerLeft.setDirection(Config.REVERSE_BLOCKER_LEFT ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        blockerRight.setDirection(Config.REVERSE_BLOCKER_RIGHT ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

        servos[0] = axon1;
        servos[1] = axon2;
        servos[2] = axon3;
        servos[3] = axon4;
        servos[4] = servo25_1;
        servos[5] = servo25_2;
        servos[6] = servo25_3;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
        imu.resetYaw();

        setServoPositions(Config.HOME);
        setBlockerPositions(Config.BLOCKER_BLOCK_POS);
    }

    private void configureMotor(DcMotor motor) {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void setServoPositions(double[] positions) {
        for (int i = 0; i < servos.length; i++) {
            servos[i].setPosition(positions[i]);
        }
    }

    public void setBlockerPositions(double position) {
        blockerLeft.setPosition(position);
        blockerRight.setPosition(position);
    }

    public void setTransferPower(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    public void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        outtakeLeft.setPower(0);
        outtakeRight.setPower(0);
        transferLeft.setPower(0);
        transferRight.setPower(0);
    }
}

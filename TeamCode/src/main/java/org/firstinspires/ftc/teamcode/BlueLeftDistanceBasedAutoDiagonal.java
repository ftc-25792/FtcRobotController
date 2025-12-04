package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Auto", group = "Aadith")
public class BlueLeftDistanceBasedAutoDiagonal extends LinearOpMode {

    // Motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;

    // Servos
    private Servo flapperLeft, flapperRight;
    private CRServo divider;

    // IMU
    private IMU imu;

    // Constants
    private static final double DRIVE_SPEED = 0.45;
    private static final double TURN_SPEED = 0.40;
    private static final double HEADING_THRESHOLD = 2.0;

    // Wheel math
    static final double WHEEL_DIAMETER_MM = 96;
    static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_MM / 25.4;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_IN;

    static final double TICKS_PER_REV = 537.7;
    static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    // Converted CM → Inches
    private static final double CM_TO_IN = 0.393701;
    private static final double DISTANCE_150CM = 150 * CM_TO_IN;
    private static final double DISTANCE_74CM = 74 * CM_TO_IN;
    private static final double DISTANCE_FORWARD_SAMPLE = 7 * CM_TO_IN;

    @Override
    public void runOpMode() {

        // Hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft  = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        flapperRight = hardwareMap.get(Servo.class, "fr");
        flapperLeft  = hardwareMap.get(Servo.class, "fl");
        divider = hardwareMap.get(CRServo.class, "sw");

        imu = hardwareMap.get(IMU.class, "imu");

        // IMU setup
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                ));
        imu.initialize(imuParameters);

        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Status: READY");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {


            driveStraight(DISTANCE_150CM, true);

            turnAngle(90, true);

            driveStraight(DISTANCE_74CM, true);

            launcherLeft.setPower(0.60);
            launcherRight.setPower(0.60);
            sleep(1000);

            for (int i = 0; i < 3; i++) {
                pickupSamples(3, 500, DISTANCE_FORWARD_SAMPLE);
                driveStraight(400 * CM_TO_IN, false);
                shoot();
                strafe(DISTANCE_74CM, false);
                driveStraight(DISTANCE_74CM, true);
            }

            launcherLeft.setPower(0);
            launcherRight.setPower(0);
            intake.setPower(0);
            stopAll();

            telemetry.addLine("AUTO COMPLETE");
            telemetry.update();
        }
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double p) {
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);
    }

    private void stopAll() { setAllPower(0); }

    // ----------------- DRIVE STRAIGHT -----------------
    private void driveStraight(double inches, boolean forward) {
        int move = (int)(inches * TICKS_PER_INCH);
        if (!forward) move = -move;

        resetEncoders();

        frontLeft.setTargetPosition(move);
        frontRight.setTargetPosition(move);
        backLeft.setTargetPosition(move);
        backRight.setTargetPosition(move);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy()  || backRight.isBusy())) {}

        stopAll();
    }

    // ----------------- STRAFE -----------------
    private void strafe(double inches, boolean left) {
        int move = (int)(inches * TICKS_PER_INCH * 1.25);

        resetEncoders();

        if (left) {
            frontLeft.setTargetPosition(-move);
            backLeft.setTargetPosition(move);
            frontRight.setTargetPosition(move);
            backRight.setTargetPosition(-move);
        } else {
            frontLeft.setTargetPosition(move);
            backLeft.setTargetPosition(-move);
            frontRight.setTargetPosition(-move);
            backRight.setTargetPosition(move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() ||
                        backLeft.isBusy()  || backRight.isBusy())) {}

        stopAll();
    }

    // ----------------- SAMPLE PICKUP -----------------
    private void pickupSamples(int count, long ms, double forwardInches) {
        for (int i = 0; i < count; i++) {
            intake.setPower(0.8);
            sleep(ms);
            intake.setPower(0);
            sleep(150);
            driveStraight(forwardInches, true);
        }
    }

    // ----------------- SHOOT -----------------
    private void shoot() {
        flapperLeft.setPosition(0.3);
        flapperRight.setPosition(0.3);
        launcherLeft.setPower(1);
        launcherRight.setPower(1);
        sleep(400);
        flapperLeft.setPosition(0);
        flapperRight.setPosition(0);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        sleep(300);
    }

    // -----------------------------------------------------------
    // IMU TURNING
    // -----------------------------------------------------------
    private double getHeading() {
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        return angle.getYaw(AngleUnit.DEGREES);
    }

    private double normalize(double a) {
        while (a > 180) a -= 360;
        while (a <= -180) a += 360;
        return a;
    }

    private boolean reached(double target) {
        double error = normalize(target - getHeading());
        return Math.abs(error) < HEADING_THRESHOLD;
    }

    private void turnAngle(double angle, boolean leftTurn) {
        imu.resetYaw();

        double start = getHeading();
        double target = normalize(start + (leftTurn ? angle : -angle));

        while (opModeIsActive() && !reached(target)) {

            double error = normalize(target - getHeading());
            double power = Math.copySign(TURN_SPEED, error);

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);
        }

        stopAll();
        sleep(200);
    }
}

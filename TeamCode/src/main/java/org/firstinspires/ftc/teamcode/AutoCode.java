package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KurryAutoFullRed", group = "Linear Opmode")
public class AutoCode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double HEADING_THRESHOLD = 2.0;

    static final double WHEEL_DIAMETER_MM = 104;
    static final double WHEEL_DIAMETER_IN = WHEEL_DIAMETER_MM / 25.4;
    static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_IN;

    static final double TICKS_PER_REV = 384.5;
    static final double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flapperRight = hardwareMap.get(Servo.class, "fr");
        flapperLeft = hardwareMap.get(Servo.class, "fl");
        divider = hardwareMap.get(CRServo.class, "sw");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParameters);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flapperRight.setDirection(Servo.Direction.REVERSE);
        flapperRight.setPosition(0.22);

        waitForStart();
        driveStraight(6,true);
        turn(45,true);

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
        frontRight.setPower(p);
        backLeft.setPower(p);
        backRight.setPower(p);
    }

    private void stopAll() {
        setAllPower(0);
        launcherLeft.setPower(0.65);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
    }

    // ---------- ENCODER DRIVE STRAIGHT ----------
    private void driveStraight(double inches, boolean isForward) {
        int move = (int)(inches * TICKS_PER_INCH);
        if (!isForward) move = -move;

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
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving", inches);
            telemetry.update();
        }

        stopAll();
    }

    private void driveBackward(double inches) {
        driveStraight(inches, false);
    }

    private void strafing(double inches, boolean isLeft) {
        double correction = 1.25;
        int move = (int)(inches * TICKS_PER_INCH * correction);

        resetEncoders();

        if (isLeft) {
            frontLeft.setTargetPosition(-move);
            frontRight.setTargetPosition(move);
            backLeft.setTargetPosition(move);
            backRight.setTargetPosition(-move);
        } else {
            frontLeft.setTargetPosition(move);
            frontRight.setTargetPosition(-move);
            backLeft.setTargetPosition(-move);
            backRight.setTargetPosition(move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Strafing", inches);
            telemetry.update();
        }

        stopAll();
    }

    // ---------- ENCODER DIAGONAL UP ----------
    private void UpDiagonalStraf(double inches, boolean IsUpperLeft) {
        int move = (int)(inches * TICKS_PER_INCH);

        resetEncoders();

        if (IsUpperLeft) {
            frontRight.setTargetPosition(move);
            backLeft.setTargetPosition(move);
        } else {
            frontLeft.setTargetPosition(move);
            backRight.setTargetPosition(move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.update();
        }

        stopAll();
    }

    private void DownDiagonalStraf(double inches, boolean IsLowerLeft) {
        int move = (int)(inches * TICKS_PER_INCH);

        resetEncoders();

        if (IsLowerLeft) {
            backRight.setTargetPosition(-move);
            frontLeft.setTargetPosition(-move);
        } else {
            frontRight.setTargetPosition(-move);
            backLeft.setTargetPosition(-move);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setAllPower(DRIVE_SPEED);

        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.update();
        }

        stopAll();
    }

    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = normalizeAngle(startAngle + (isLeft ? angle : -angle));

        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            double error = normalizeAngle(targetAngle - getHeading());

            double scale = (Math.abs(error) > 10) ? 1.0 : 0.5;
            double leftPower = -TURN_SPEED * Math.signum(error) * scale;
            double rightPower = TURN_SPEED * Math.signum(error) * scale;

            frontLeft.setPower(leftPower);
            backLeft.setPower(leftPower);
            frontRight.setPower(rightPower);
            backRight.setPower(rightPower);

            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        setAllPower(0);
        sleep(300);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = normalizeAngle(getHeading());
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD;
    }

    void flapper() {
        flapperRight.setPosition(0.33);
        sleep(1000);
        flapperRight.setPosition(0.67);
        sleep(1000);
        flapperRight.setPosition(0.33);
        sleep(1000);
        flapperLeft.setPosition(0.02);
        sleep(1000);
        flapperLeft.setPosition(0.28);
        sleep(1000);
        launcherLeft.setPower(0.65);
        launcherRight.setPower(0.65);
        intake.setPower(1.0);
    }

    private void shoot(boolean IsRight) {
        if(IsRight) {
            launcherRight.setPower(0.8);
        } else{
            launcherLeft.setPower(0.8);
        }
        sleep(2500);
    }

}
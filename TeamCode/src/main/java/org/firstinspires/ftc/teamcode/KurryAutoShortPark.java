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

@Autonomous(name = "KurryAutoShortPark", group = "Linear Opmode")
public class KurryAutoShortPark extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private IMU imu;
    private ElapsedTime runtime = new ElapsedTime();
    private Servo flapperRight, flapperLeft;
    private CRServo divider;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.5;
    private static final double HEADING_THRESHOLD = 1.0;

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
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
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
        resetEncoders();

        // Your autonomous path (unchanged)
        driveStraight(15, false);
        turn(135,false);
        driveStraight(15,true);


//        turn(90, true);
//        turn(90,false);
//        driveStraight(36, true);
//        driveStraight(36, false);
//        turn(90, false);
//        driveStraight(18, false);
//        turn(45, false);
//        turn(45, true);
//        driveStraight(12, true);
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
    }

    // ---------- DRIVE STRAIGHT ----------
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

        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {

            telemetry.addData("Driving", inches);
            telemetry.update();
        }

        stopAll();
        sleep(100);
    }

    private void strafing(double inches, boolean isLeft) {
        double correction = 1.12;
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

        runtime.reset();
        while (opModeIsActive() &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy()) &&
                runtime.seconds() < 5) {

            telemetry.addData("Strafing", inches);
            telemetry.update();
        }

        stopAll();
        sleep(100);
    }
    private void turn(double angle, boolean isLeft) {
        double target = normalizeAngle(getHeading() + (isLeft ? angle : -angle));

        while (opModeIsActive()) {

            double current = getHeading();
            double error = normalizeAngle(target - current);

            if (Math.abs(error) < HEADING_THRESHOLD)
                break;

            double kP = 0.012;
            double power = kP * error;

            power = Math.max(-TURN_SPEED, Math.min(TURN_SPEED, power));

            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Turn Target", target);
            telemetry.addData("Heading", current);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        stopAll();
        sleep(100);
    }

    private double getHeading() {
        YawPitchRollAngles yp = imu.getRobotYawPitchRollAngles();
        return yp.getYaw(AngleUnit.DEGREES);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutoCodeMeet", group = "Yippee")
public class RedNonGoalStarting extends LinearOpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;
    private DcMotor launcherLeft, launcherRight, intake;
    private DistanceSensor sensorDistance;
    private IMU imu;

    private final double DRIVE_POWER = 0.4;
    private final double LAUNCH_POWER = 0.60;
    private final long SPINUP_TIME_MS = 1000;
    private final double DISTANCE_150CM = 150;
    private final double DISTANCE_74CM = 74;
    private final double DISTANCE_FORWARD_PER_SAMPLE = 7;
    private final long INTAKE_MS = 500;
    private final int LOOP_COUNT = 3;
    private final double HEADING_THRESHOLD = 2.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        imu = hardwareMap.get(IMU.class, "imu");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        launcherLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(imuParameters);

        telemetry.addLine("Ready");
        telemetry.update();
        waitForStart();

        waitForStart(); {

            driveForwardDistance(DRIVE_POWER, DISTANCE_150CM);

            turnToAngle(0.4, 90, true);

            driveForwardDistance(DRIVE_POWER, DISTANCE_74CM);

            launcherLeft.setPower(LAUNCH_POWER);
            sleep(SPINUP_TIME_MS);

            for (int i = 0; i < LOOP_COUNT; i++) {
                pickupSamples(3, INTAKE_MS, DISTANCE_FORWARD_PER_SAMPLE);
                backwards(0.5, 400);
                shoot();
                strafeRightDistance(0.5, DISTANCE_74CM);
                driveForwardDistance(DRIVE_POWER, DISTANCE_74CM);
            }

            launcherLeft.setPower(0);
            launcherRight.setPower(0);
            intake.setPower(0);
            setAllPower(0);

            telemetry.addLine("Auto complete");
            telemetry.update();
        }
    }
    private void setAllPower(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    private void backwards(double power, double distanceCm) {
        double startDistance = sensorDistance.getDistance(DistanceUnit.CM);
        double distanceTraveled = 0;

        while (opModeIsActive() && distanceTraveled < distanceCm) {
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(-power);
            backRight.setPower(-power);

            double currentDistance = sensorDistance.getDistance(DistanceUnit.CM);
            distanceTraveled = startDistance - currentDistance;

            telemetry.addData("Strafe traveled (cm)", "%.1f / %.1f", distanceTraveled, distanceCm);
            telemetry.update();
        }
        setAllPower(0);
    }
    private void pickupSamples(int count, long ms, double forwardCm) {
        for (int i = 0; i < count; i++) {
            intake.setPower(0.8);
            sleep(ms);
            intake.setPower(0);
            sleep(150);
            driveForwardDistance(DRIVE_POWER, forwardCm);
        }
    }

    private void shoot() {
        launcherLeft.setPower(1);
        launcherRight.setPower(1);
        sleep(400);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        sleep(300);
    }

    private void driveForwardDistance(double power, double distanceCm) {
        double startDistance = sensorDistance.getDistance(DistanceUnit.CM);
        double distanceTraveled = 0;
        while (opModeIsActive() && distanceTraveled < distanceCm) {
            setAllPower(power);
            double currentDistance = sensorDistance.getDistance(DistanceUnit.CM);
            distanceTraveled = startDistance - currentDistance;
            telemetry.addData("Distance traveled (cm)", "%.1f / %.1f", distanceTraveled, distanceCm);
            telemetry.update();
        }
        setAllPower(0);
    }
    private void turnToAngle(double power, double targetAngle, boolean isLeft) {

        double degreesToTurn = isLeft ? -targetAngle : targetAngle;

        double startAngle = getHeading();
        double desiredAngle = startAngle + degreesToTurn;

        while (desiredAngle >= 180) desiredAngle -= 360;
        while (desiredAngle < -180) desiredAngle += 360;

        telemetry.addData("Turn start", "%.2f", startAngle);
        telemetry.addData("Turning to", "%.2f", desiredAngle);
        telemetry.update();

        boolean turnRight = degreesToTurn > 0;
        double turnPower = Math.abs(power);

        if (turnRight) {
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);
        } else {
            frontLeft.setPower(-turnPower);
            backLeft.setPower(-turnPower);
            frontRight.setPower(turnPower);
            backRight.setPower(turnPower);
        }
        while (opModeIsActive() && !isAngleReached(desiredAngle)) {
            telemetry.addData("Heading", "%.2f", getHeading());
            telemetry.addData("Target", "%.2f", desiredAngle);
            telemetry.update();
        }

        setAllPower(0);
        sleep(200);
    }

    private void strafeRightDistance(double power, double distanceCm) {
        double startDistance = sensorDistance.getDistance(DistanceUnit.CM);
        double distanceTraveled = 0;

        while (opModeIsActive() && distanceTraveled < distanceCm) {
            frontLeft.setPower(power);
            backLeft.setPower(-power);
            frontRight.setPower(-power);
            backRight.setPower(power);

            double currentDistance = sensorDistance.getDistance(DistanceUnit.CM);
            distanceTraveled = startDistance - currentDistance;

            telemetry.addData("Strafe traveled (cm)", "%.1f / %.1f", distanceTraveled, distanceCm);
            telemetry.update();
        }
        setAllPower(0);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double current = getHeading();
        double desiredAngle = targetAngle - current;
        while (desiredAngle > 180) desiredAngle -= 360;
        while (desiredAngle< -180) desiredAngle += 360;
        return Math.abs(desiredAngle) < HEADING_THRESHOLD;
    }
}

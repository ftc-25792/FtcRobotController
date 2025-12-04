package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "ThorAuto", group = "Linear Opmode")
public class ThorWithoutShootingMechanism extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private IMU imu;

    private static final double DRIVE_SPEED = 0.5;
    private double STRAFE_DISTANCE = 0.5;
    static final double TURN_SPEED = 0.4;
    private final double HEADING_THRESHOLD = 2.0;
    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");



        // --- Motor Directions ---
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        if (opModeIsActive()) {
            driveStraight(1, false);

            sleep(1000);

            turn(90, false);

            for (int i = 0; i < 3; i++, STRAFE_DISTANCE =+ 0.5) {
                driveStraight(0.2, true);

                driveStraight(0.2, false);

                sleep(500);

                strafing(STRAFE_DISTANCE, true);
            }
        }
    }

    private void turn(double angle, boolean isLeft) {
        imu.resetYaw();
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        // Set motors for turning
        if (isLeft) {
            leftFront.setPower(-TURN_SPEED);
            leftRear.setPower(-TURN_SPEED);
            rightFront.setPower(TURN_SPEED);
            rightRear.setPower(TURN_SPEED);
        } else {
            leftFront.setPower(TURN_SPEED);
            leftRear.setPower(TURN_SPEED);
            rightFront.setPower(-TURN_SPEED);
            rightRear.setPower(-TURN_SPEED);
        }
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after turning
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

    private void driveStraight(double distance, boolean isForward) {
        if (isForward) setMotorPower(DRIVE_SPEED);
        else setMotorPower(-DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);

        sleep(500);
    }

    private void strafing(double distance, boolean isRight) {
        if (isRight) {
            leftFront.setPower(DRIVE_SPEED);
            rightFront.setPower(-DRIVE_SPEED);
            leftRear.setPower(-DRIVE_SPEED);
            rightRear.setPower(DRIVE_SPEED);
        } else {
            leftFront.setPower(-DRIVE_SPEED);
            rightFront.setPower(DRIVE_SPEED);
            leftRear.setPower(DRIVE_SPEED);
            rightRear.setPower(-DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED));
        setMotorPower(0);

        sleep(1000);
    }


    private void setMotorPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

}

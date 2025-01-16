package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KammanderAuto NO PARK", group = "Autonomous")
public class KammanderAuto_N0_PARK extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private IMU imu;
    private DcMotor armMotor;
    private DcMotor viperMotor;

    // Define constants

    double armPosition = 0;
    double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;
    static final double HEADING_THRESHOLD = 1.0;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double hover = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_IN_HIGH = 85 * ARM_TICKS_PER_DEGREE;
    final double hover2 = 15 + ARM_TICKS_PER_DEGREE;
    private CRServo intake;
    private Servo wrist;
    final double INTAKE_OFF = 0;
    final double INTAKE_ON = 0.69;
    int VIPER_HIGH_BASKET = -3000;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        intake = hardwareMap.get(CRServo.class, "Intake");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");
        wrist = hardwareMap.get(Servo.class, "Wrist");
        // Set motor directions
        wrist.setDirection(Servo.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(3, CurrentUnit.AMPS);
        intake.setPower(INTAKE_OFF);

        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        ((DcMotorEx) viperMotor).setCurrentAlert(4, CurrentUnit.AMPS);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        armControls(hover,0.5);
        sleep(500);
        wrist.setPosition(0.55);
        strafing(0.3, false);
        armControls(ARM_SCORE_IN_HIGH,0.3);
        sleep(2000);
        wrist.setPosition(0.55);
        driveStraight(0.5, true);
        turn(42,true);
        driveStraight(0.03,false);
        sleep(250);
        moveslide(VIPER_HIGH_BASKET);
        sleep(1000);
        wrist.setPosition(0.55);

        sleep(1000);
        intake.setPower(INTAKE_ON);
        sleep(100);
        driveStraight(0.02,false);
        sleep(1500);
        moveslide(0);
        sleep(1800);
        armControls(hover,0.3);
        sleep(250);
        turn(296,true);
        driveStraight(0.12 ,false);
        strafing(0.83,false);
        driveStraight(0.11,true);
        strafing(0.85,true);
        UpDiagonalStraf(0.45,true);
        DownDiagonalStraf(0.45,false);
        driveStraight(0.2,false);
        strafing(0.2, true);
        driveStraight(0.4, true);

//        driveBackward(0.77);
//        strafing(0.5,true);

    }
    void moveslide(int viperposition) {
        telemetry.addData("Slide Value", viperposition);
        telemetry.update();
        viperMotor.setTargetPosition(viperposition);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setPower(0.65);
    }
    private void driveBackward(double distance) {
//        // Start moving forward or backward based on direction
//        double initialHeading = getHeading();



        setMotorPower(-0.69-21);


        // Calculate the distance (in meters)
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors

        // loop to maintain heading while driving
//        while (opModeIsActive() && (System.currentTimeMillis() < endTime)) {
//            double correction = getSteeringCorrection(initialHeading, 0.03);
//            setMotorPower(DRIVE_SPEED - correction);
//        }

        setMotorPower(0);
        sleep(500);
    }

    private void UpDiagonalStraf(double distance, boolean IsUpperLeft){
        if (IsUpperLeft == true) {
            rightFrontMotor.setPower(DRIVE_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED);

        }else{
            rightRearMotor.setPower(DRIVE_SPEED);
            leftFrontMotor.setPower(DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward

    }
    private void DownDiagonalStraf(double distance, boolean IsLowerLeft){
        if (IsLowerLeft== true) {
            rightRearMotor.setPower(-DRIVE_SPEED);
            leftFrontMotor.setPower(-DRIVE_SPEED);


        }else{
            rightFrontMotor.setPower(-DRIVE_SPEED);
            leftRearMotor.setPower(-DRIVE_SPEED);
        }
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving forward
    }
    private void armControls(double position, double speed){

        if (position == ARM_SCORE_IN_HIGH){
            armPosition = ARM_SCORE_IN_HIGH;
        } else if (position == hover) {
            armPosition = hover;
        } else if (position == hover2){
            armPosition = hover;
        }

        armMotor.setTargetPosition((int) (armPosition));
        ((DcMotorEx) armMotor).setPower(speed);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    private void driveStraight(double distance, boolean isForward) {
//        // Start moving forward or backward based on direction
//        double initialHeading = getHeading();

        if (isForward == true) {
            setMotorPower(DRIVE_SPEED);
        } else {
            setMotorPower(-DRIVE_SPEED);
        }

        // Calculate the distance (in meters)
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors

        // loop to maintain heading while driving
//        while (opModeIsActive() && (System.currentTimeMillis() < endTime)) {
//            double correction = getSteeringCorrection(initialHeading, 0.03);
//            setMotorPower(DRIVE_SPEED - correction);
//        }

        setMotorPower(0);
        sleep(500);
    }

    private void turn(double angle, boolean isLeft) {
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;

        // Set motors for turning
        if (isLeft) {
            leftFrontMotor.setPower(-TURN_SPEED);
            leftRearMotor.setPower(-TURN_SPEED);
            rightFrontMotor.setPower(TURN_SPEED);
            rightRearMotor.setPower(TURN_SPEED);
        } else {
            leftFrontMotor.setPower(TURN_SPEED);
            leftRearMotor.setPower(TURN_SPEED);
            rightFrontMotor.setPower(-TURN_SPEED);
            rightRearMotor.setPower(-TURN_SPEED);
        }

        // Continue turning until the target angle is reached
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
        }

        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after turning
    }

    private void strafing(double distance, boolean isLeft) {
        // Set powers for strafing
        if (isLeft) {
            leftFrontMotor.setPower(-DRIVE_SPEED);
            rightFrontMotor.setPower(DRIVE_SPEED);
            leftRearMotor.setPower(DRIVE_SPEED);
            rightRearMotor.setPower(-DRIVE_SPEED);


        } else {
            leftFrontMotor.setPower(DRIVE_SPEED);
            rightFrontMotor.setPower(-DRIVE_SPEED);
            leftRearMotor.setPower(-DRIVE_SPEED);
            rightRearMotor.setPower(DRIVE_SPEED);
        }

        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjusting time based on speed
        setMotorPower(0); // Stop all motors
        sleep(500); // Pause briefly after moving
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();
        // Normalize current angle
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        return Math.abs(currentAngle - targetAngle) < HEADING_THRESHOLD; // Tolerance check
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    // Method to get steering correction
    private double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        double headingError = desiredHeading - getHeading();

        // Normalize the error
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return headingError * proportionalGain; // Proportional controller
    }
}

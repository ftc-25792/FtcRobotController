package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KammanderWithTeleMotors", group = "Combined")
public class AutoRough extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;

    // Intake & launcher (from teleop)
    private DcMotor Motor1, Motor2, Motor3;

    // Servos
    private CRServo intake;
    private Servo servo2, servo3, wrist;

    // IMU
    private IMU imu;

    // Autonomous constants
    double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;
    static final double HEADING_THRESHOLD = 1.0;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double hover = 10 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_IN_HIGH = 90 * ARM_TICKS_PER_DEGREE;
    final double hover2 = 15 + ARM_TICKS_PER_DEGREE;
    final double INTAKE_OFF = 0;
    final double INTAKE_ON = 0.69;
    int VIPER_HIGH_BASKET = -3000;
    int PickUp = -700;
    double armPosition = 0;

    @Override
    public void runOpMode() {

        // ----------------- Hardware Initialization -----------------
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");

        Motor1 = hardwareMap.get(DcMotor.class, "Intake");
        Motor2 = hardwareMap.get(DcMotor.class, "LauncherRight");
        Motor3 = hardwareMap.get(DcMotor.class, "LauncherLeft");



        servo2 = hardwareMap.get(Servo.class, "Servo2");
        servo3 = hardwareMap.get(Servo.class, "Servo3");
        intake = hardwareMap.get(CRServo.class, "Servo1");

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // Drive motors directions
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        // Motor modes
        Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        servo2.setPosition(0);
        servo3.setPosition(0);
        intake.setPower(INTAKE_OFF);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
//
//        // ----------------- Autonomous Routine -----------------
//        // Example usage of teleop motors in auto:
//        // Start launcher motors
//        Motor1.setPower(1);
//        Motor2.setPower(1);
//        Motor3.setPower(1);
//        sleep(1000); // Run for 1 second
//        Motor1.setPower(0);
//        Motor2.setPower(0);
//        Motor3.setPower(0);
//        sleep(1500);

        // Example: move robot forward
        driveStraight(0.5, true);
        sleep(500);
        turn(45,false);



//        // Fire intake/launcher again
//        Motor1.setPower(1);
//        Motor2.setPower(1);
//        Motor3.setPower(1);
//        sleep(500);
//        Motor1.setPower(0);
//        Motor2.setPower(0);
//        Motor3.setPower(0);

        telemetry.addData("Status", "Auto Finished");
        telemetry.update();
    }
    private void driveBackward(double distance) {



        // Calculate the distance (in meters)
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        setMotorPower(0);
        sleep(500);
    }
    private void launch(double power, boolean IsGreen){
        
        if (IsGreen == true){
            Motor1.setPower(power);
        } else {
            Motor2.setPower(power);
        }
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

    private void driveStraight(double distance, boolean isForward) {

        if (isForward == true) {
            setMotorPower(DRIVE_SPEED);
        } else {
            setMotorPower(-DRIVE_SPEED);
        }

        // Calculate the distance (in meters)
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors



        setMotorPower(0);
        sleep(500);
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

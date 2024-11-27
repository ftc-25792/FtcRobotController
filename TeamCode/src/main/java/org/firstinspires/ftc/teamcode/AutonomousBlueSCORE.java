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

@Autonomous(name = "AUTO CODE FOR SCORING", group = "Autonomous")
public class AutonomousBlueSCORE extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private IMU imu;
    DcMotor armMotor;
    CRServo intake; // Intaking servo
    Servo wrist; // Wrist servo
    DcMotor viperMotor;
    private static final double FORWARD_DISTANCE = 2.; // Adjust distance in meters
    private static final double TURN_ANGLE = -90.0; // Degrees to turn
    private static final double DRIVE_SPEED = 0.5; // Speed for driving
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360; // Encoder ticks per degree (assumes specific motor characteristics)
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 4 * ARM_TICKS_PER_DEGREE;//250
    final double ARM_SCORE_SAMPLE_IN_LOW = 30 * ARM_TICKS_PER_DEGREE; //160
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments
    final double ARM_SCORE_IN_HIGH = 38 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "Left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "Right_front");
        leftRearMotor = hardwareMap.get(DcMotor.class, "Left_rear");
        rightRearMotor = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(-0.8333); // Folded in position

        // Set motor directions
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD; //BACK
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();


        int LoopCount = 10;

        // Drive in a squarefor (int i = 1; i <= LoopCount; i++)


        telemetry.update();
        driveForward(0.05);
        armMotor.setPower(1);
        sleep(50);
        wrist.setPosition(-0.833);
        turn(TURN_ANGLE);
        driveForward(0.5);
        armMotor.setPower(ARM_SCORE_IN_HIGH);
        sleep(100);
        viperMotor.setPower(0.2);
        sleep(400);
        intake.setPower(-1);
        viperMotor.setPower(-0.2);
        sleep(400);
        driveForward(-FORWARD_DISTANCE);
        driveForward(-FORWARD_DISTANCE);
    }

    private void driveForward(double distance) {
        setMotorPower(DRIVE_SPEED);
        sleep((long) (distance * 1000 / DRIVE_SPEED)); // Adjust time based on speed
        setMotorPower(0); // Stop all motors
        sleep(1500); // Pause briefly after moving forward
    }

    private void turn(double angle) {
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;

        // Normalize the angle to keep it within -180 to 180 degrees
        while (targetAngle >= 180) targetAngle -= 360;
        while (targetAngle < -180) targetAngle += 360;


        double power = 0.1;

        // Set motors for turning
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        rightRearMotor.setPower(-power);

        // Continue turning until the target angle is reached
        while (opModeIsActive() && !isAngleReached(targetAngle)) {
            telemetry.addData("Current Angle", getHeading());
            telemetry.addData("Start ANgele",startAngle);
            telemetry.addData("Target Angle", targetAngle);
            telemetry.update();
            // sleep(1000);
        }

        setMotorPower(0); // Stop all motors
        sleep(1800); // Pause briefly after turning
    }

    private double getHeading() {
        // Get the IMU's heading
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES); // Return the Z-axis angle
    }

    private boolean isAngleReached(double targetAngle) {
        double currentAngle = getHeading();

        // Normalize the angle to keep it within -180 to 180 degrees
        while (currentAngle >= 180) currentAngle -= 360;
        while (currentAngle < -180) currentAngle += 360;
        double error = currentAngle - targetAngle;




        return Math.abs(error) < 1.0; // Tolerance can be adjusted
    }

    private void setMotorPower(double power) {
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }
}
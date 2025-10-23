package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU; // Modern IMU interface
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "KurryForward56byIMU", group = "Autonomous")
public class KurryForward56byIMU extends LinearOpMode {

    // --- DECLARATIONS ---
    public DcMotor leftFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightRearMotor;
    public IMU imu; // Using the modern IMU interface

    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double HEADING_GAIN = 0.04;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        // NOTE: Standard FTC practice reverses the left side for a typical chassis
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU Configuration
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Encoder Reset
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run Mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        // --- EXECUTION ---
        if (opModeIsActive()) {
            driveStraight(56, 0.5); // Call the function defined below
        }
    }

    // --- HELPER FUNCTIONS (DEFINED OUTSIDE runOpMode) ---

    public void driveStraight(double inches, double power) {
        int newLeftTarget;
        int newRightTarget;
        float heading = getHeading();

        // Calculate targets
        newLeftTarget = leftFrontMotor.getCurrentPosition() + (int)Math.round(inches * COUNTS_PER_INCH);
        newRightTarget = rightFrontMotor.getCurrentPosition() + (int)Math.round(inches * COUNTS_PER_INCH);

        // Set targets
        leftFrontMotor.setTargetPosition(newLeftTarget);
        leftRearMotor.setTargetPosition(newLeftTarget);
        rightFrontMotor.setTargetPosition(newRightTarget);
        rightRearMotor.setTargetPosition(newRightTarget);

        // Set to RUN_TO_POSITION
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set initial power
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);

        // Loop for steering correction
        while(opModeIsActive() && (leftFrontMotor.isBusy() || rightFrontMotor.isBusy())) {
            float currentAngle = getHeading();
            float error = heading - currentAngle;
            double steering = error * HEADING_GAIN;

            // CORRECTED POWER LOGIC: Add steering to one side, subtract from the other
            double leftPower = power + steering;
            double rightPower = power - steering;

            leftPower = Range.clip(leftPower, -1, 1);
            rightPower = Range.clip(rightPower, -1, 1);

            // Apply corrected power
            leftFrontMotor.setPower(leftPower);
            leftRearMotor.setPower(leftPower);
            rightFrontMotor.setPower(rightPower);
            rightRearMotor.setPower(rightPower);

            // NOTE: The stop and reset code was REMOVED from the loop!
        }

        // --- CLEANUP (EXECUTED ONCE AFTER LOOP FINISHES) ---
        // Stop all motion
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);

        // Set run mode back to RUN_USING_ENCODER
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public float getHeading() {
        // Modern IMU uses YawPitchRollAngles object to get rotation
        return (float) imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
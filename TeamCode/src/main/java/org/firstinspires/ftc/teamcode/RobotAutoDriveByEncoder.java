package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")

public class RobotAutoDriveByEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.77952756; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");

        // Set motor directions
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders
        resetEncoders();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at FL FR BL BR", "%7d :%7d :%7d :%7d",
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition());



        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Drive forward, turn right, and then drive backward
        encoderDrive(DRIVE_SPEED, 24, 24,5.0);  // Forward 48 inches
        /*encoderDrive(TURN_SPEED, 12, -12, 4.0);  // Turn Right 12 inches
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0); // Backward 24 inches*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    private void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTargetFront;
        int newLeftTargetBack;
        int newRightTargetFront;
        int newRightTargetBack;

        if (opModeIsActive()) {
            // Calculate new target position for all four motors
            newLeftTargetFront = motorFrontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTargetBack = motorBackLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetFront = motorFrontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTargetBack = motorBackRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            // Set target positions
            motorFrontLeft.setTargetPosition(newLeftTargetFront);
            motorBackLeft.setTargetPosition(newLeftTargetBack);
            motorFrontRight.setTargetPosition(newRightTargetFront);
            motorBackRight.setTargetPosition(newRightTargetBack);

            // Turn On RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset timeout time and start motion.
            runtime.reset();
            motorFrontLeft.setPower(Math.abs(speed));
            motorBackLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFrontLeft.isBusy() && motorBackLeft.isBusy() && motorFrontRight.isBusy() && motorBackRight.isBusy())) {

                telemetry.addData("Running to", " %7d :%7d :%7d :%7d ",
                        newLeftTargetFront, newRightTargetFront, newLeftTargetBack, newRightTargetBack);
                telemetry.addData("Currently at FL FR BL BR", " at %7d :%7d %7d : %7d",
                        motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250); // Optional pause after each move.
        }
    }
}

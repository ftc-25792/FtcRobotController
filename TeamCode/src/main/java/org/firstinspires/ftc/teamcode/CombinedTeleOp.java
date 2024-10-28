
/* Copyright (c) 2017 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 * ...
 * (Include the entire license text here, from the first class)
 * ...
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="CombinedTeleOpWithArmCorrection", group="Linear Opmode")
public class CombinedTeleOp extends LinearOpMode {
    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;
    Servo intake; // Intaking servo
    Servo wrist; // Wrist servo

    // Arm control variables
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360; // Encoder ticks per degree (assumes specific motor characteristics)
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    // PID control variables for arm
    double kp = 0.02; // Proportional gain
    double ki = 0.0;  // Integral gain
    double kd = 0.01; // Derivative gain
    double lastError = 0.0; // Last error value for derivative calculation
    double integral = 0.0; // Integral error
    int targetArmPosition = 0; // Target position

    @Override
    public void runOpMode() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");


        // Set motor directions
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set arm motor to use encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) armMotor).setCurrentAlert(2, CurrentUnit.AMPS);

        // Initialize servos
        intake = hardwareMap.get(Servo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setPosition(0.8333); // Folded in position

        // Wait for the game driver to press play
        waitForStart();

        while (opModeIsActive()) {
            // Handle motor control
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;

            // Calculate wheel powers
            double frontLeftPower = leftStickY + leftStickX;
            double frontRightPower = leftStickY - leftStickX;
            double backLeftPower = leftStickY + rightStickX;
            double backRightPower = leftStickY - rightStickX;

            // Set motor powers
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

            // Handle arm position and control
            if (gamepad1.right_bumper) {
                armPosition = ARM_COLLECT;
                wrist.setPosition(0.5); // Open position
            } else if (gamepad1.y) {
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            } else if (gamepad1.dpad_left) {
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                wrist.setPosition(0.8333); // Folded in position
            }

            // Fine-tune position using triggers
            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger - gamepad1.left_trigger);
            targetArmPosition = (int) (armPosition + armPositionFudgeFactor);

            // PID control logic
            int currentArmPosition = armMotor.getCurrentPosition();
            double error = targetArmPosition - currentArmPosition;
            integral += error; // Accumulate the integral term
            double derivative = error - lastError;
            double output = Range.clip((kp * error + ki * integral + kd * derivative), -1.0, 1.0);

            // Set the arm motor power based on PID output
            armMotor.setPower(output);

            // Update last error for the next cycle
            lastError = error;

            // Telemetry for debugging
            telemetry.addData("Target Arm Position", targetArmPosition);
            telemetry.addData("Current Arm Position", currentArmPosition);
            telemetry.update();
        }
    }
}

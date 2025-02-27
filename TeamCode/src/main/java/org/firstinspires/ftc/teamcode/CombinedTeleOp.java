/* Copyright (c) 2017 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CombinedTeleOpWithHoldingArmPosition", group="Linear Opmode")
public class CombinedTeleOp extends OpMode {
    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;
    DcMotor viperMotor; // For the Viper slide
    IMU imu;

    // Servo variables
    Servo[] servo;
    static final double INCREMENT = 0.1; // Amount to increment/decrement servo position
    static final double MAX_POS = 1.0; // Maximum servo position
    static final double MIN_POS = 0.0; // Minimum servo position
    double[] positions; // Current positions for each servo

    // Arm control variables
    int targetArmPosition = 0; // Target position for the arm
    boolean armMoving = false; // Flag to track if the arm is moving

    // PID control variables
    double kp = 0.02; // Proportional gain
    double ki = 0.0;  // Integral gain (not used in this basic implementation)
    double kd = 0.01; // Derivative gain
    double lastError = 0.0; // Last error value for derivative calculation
    double integral = 0.0; // Integral error (optional for more stability)

    // Arm control states
    enum ArmState {
        MOVING_UP,
        MOVING_DOWN,
        HOLDING
    }

    ArmState armState = ArmState.HOLDING; // Initial state

    @Override
    public void init() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");

        // Set motor directions
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motor to use encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Initialize servos
        int SERVO_COUNT = 4;
        servo = new Servo[SERVO_COUNT];
        positions = new double[SERVO_COUNT]; // Array to hold positions for each servo
        for (int i = 0; i < SERVO_COUNT; i++) {
            servo[i] = hardwareMap.get(Servo.class, "servo" + (i + 1));
            positions[i] = MIN_POS; // Start all servos at minimum position
            servo[i].setPosition(positions[i]);
        }
    }

    @Override
    public void loop() {
        // Handle motor control
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;
        boolean triangle = gamepad1.y; // Button to move arm up
        boolean aButton = gamepad1.a; // Button to retract arm
        boolean rightTrigger = gamepad1.right_trigger > 0.9; // Trigger to extend Viper slide
        boolean leftTrigger = gamepad1.left_trigger > 0.9; // Trigger to retract Viper slide
        boolean right = gamepad1.dpad_right; // Button to increment servo position
        boolean left = gamepad1.dpad_left; // Button to decrement servo position

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

        // Control arm with triangle button (up) and A button (down)
        if (triangle) {
            armState = ArmState.MOVING_UP;
            targetArmPosition += 20; // Increment target position (adjust as needed)
            armMoving = true; // Set moving flag
        } else if (aButton) {
            armState = ArmState.MOVING_DOWN;
            targetArmPosition -= 20; // Decrement target position (adjust as needed)
            armMoving = true; // Set moving flag
        } else {
            armState = ArmState.HOLDING;
        }

        // Get the current position
        int currentPosition = armMotor.getCurrentPosition();

        // Control logic based on state
        switch (armState) {
            case MOVING_UP:
                armMotor.setPower(0.25); // Move up at full power
                break;

            case MOVING_DOWN:
                armMotor.setPower(-0.15); // Move down at full power
                break;

            case HOLDING:
                // Calculate PID output
                double error = targetArmPosition - currentPosition;
                integral += error; // Accumulate integral
                double derivative = error - lastError;
                double output = (kp * error) + (kd * derivative);

                // Set motor power based on PD output
                armMotor.setPower(Range.clip(output, -1.0, 1.0)); // Clip power to valid range

                // Update last error
                lastError = error;

                // Optional: Stop if within a small threshold
                if (Math.abs(error) < 2) { // Threshold
                    armMotor.setPower(0.0); // Stop motor if close enough
                }
                break;
        }

        // Control Viper slide
        if (rightTrigger) {
            viperMotor.setPower(1.0); // Extend Viper slide
        } else if (leftTrigger) {
            viperMotor.setPower(-1.0); // Retract Viper slide
        } else {
            viperMotor.setPower(0.0); // Stop Viper slide
        }

        // Control servo positions with buttons
        if (right) { // Increment position
            for (int i = 0; i < servo.length; i++) {
                if (positions[i] < MAX_POS) {
                    positions[i] += INCREMENT;
                    if (positions[i] > MAX_POS) {
                        positions[i] = MAX_POS; // Clamp to max
                    }
                    servo[i].setPosition(positions[i]);
                }
            }
        }

        if (left) { // Decrement position
            for (int i = 0; i < servo.length; i++) {
                if (positions[i] > MIN_POS) {
                    positions[i] -= INCREMENT;
                    if (positions[i] < MIN_POS) {
                        positions[i] = MIN_POS; // Clamp to min
                    }
                    servo[i].setPosition(positions[i]);
                }
            }
        }

        // Telemetry for debugging
        for (int i = 0; i < servo.length; i++) {
            telemetry.addData("Servo " + (i + 1) + " Position", "%5.2f", positions[i]);
        }
        telemetry.addData("Arm Target Position", targetArmPosition);
        telemetry.addData("Arm Current Position", currentPosition);
        telemetry.update();
    }
}

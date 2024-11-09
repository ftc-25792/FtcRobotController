

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="SERVO TELE OP", group="Linear Opmode")
public class CombinedTeleOp2 extends LinearOpMode {
    // Declare motors
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor armMotor;
    CRServo intake; // Intaking servo
    Servo wrist; // Wrist servo
    DcMotor viperMotor;

    // Arm control variables
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360; // Encoder ticks per degree (assumes specific motor characteristics)
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 10 * ARM_TICKS_PER_DEGREE;//250
    final double ARM_SCORE_SAMPLE_IN_LOW = 30 * ARM_TICKS_PER_DEGREE; //160
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE; // Allow for slight adjustments
    final double ARM_SCORE_IN_HIGH = 40 * ARM_TICKS_PER_DEGREE;

    //Viper Slide Control Variables
    final double VIPERSLIDE_TICKS_PER_DEGREE = 537.69 / 360;
    final double VIPER_COLLECT = 50 * VIPERSLIDE_TICKS_PER_DEGREE;


    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 50 * ARM_TICKS_PER_DEGREE;//160
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 0 * ARM_TICKS_PER_DEGREE;//15  //8
    final double INTAKE_COLLECT = 1;
    final double INTAKE_OFF = 0;
    final double INTAKE_DEPOSIT = -1;


    @Override
    public void runOpMode() {
        // Initialize motors
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");


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

        // In
        // itialize servos
        intake = hardwareMap.get(CRServo.class, "intake");
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


            boolean rightTrigger = false;
            boolean leftTrigger = false;
            if (rightTrigger) {
                armMotor.setPower(ARM_COLLECT);
            } else if (leftTrigger) {
                armMotor.setPower(ARM_SCORE_IN_HIGH);
            } else {
                armMotor.setPower(0.0);
            }

            if (gamepad2.b) {
                viperMotor.setPower(1); // Extend Viper slide
            } else if (gamepad2.x) {
                viperMotor.setPower(-1); // Retract Viper slide
            } else {
                viperMotor.setPower(0); // Stop Viper slide
            }
            if (gamepad1.left_bumper) {
                intake.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                intake.setPower(-0.5);
            } else if (gamepad1.a)
            {
                intake.setPower(0);
            }

            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust (or "fudge") the arm position slightly with the gamepad triggers.
            We want the left trigger to move the arm up, and right trigger to move the arm down.
            So we add the right trigger's variable to the inverse of the left trigger. If you pull
            both triggers an equal amount, they cancel and leave the arm at zero. But if one is larger
            than the other, it "wins out". This variable is then multiplied by our FUDGE_FACTOR.
            The FUDGE_FACTOR is the number of degrees that we can adjust the arm by with this function. */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));


            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setPower(0.1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()) {
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();

        }
    }
}
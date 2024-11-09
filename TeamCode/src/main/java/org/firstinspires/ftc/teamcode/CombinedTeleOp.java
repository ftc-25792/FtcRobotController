package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="CombinedTeleOpWithArmCorrection", group="Linear Opmode")
public class CombinedTeleOp extends LinearOpMode {
    // Declare motors and servos
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
    DcMotor viperMotor, armMotor;
    Servo intake, wrist;

    // Arm control variables
    final double FINAL_EXTENDED_POSITION = 25;
    final double ARM_TICKS_PER_DEGREE = 7125.16 / 360;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_COLLECT = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT = 20 * ARM_TICKS_PER_DEGREE;

    final double INTAKE_COLLECT = -1.0;
    final double INTAKE_OFF = 0.0;
    final double INTAKE_DEPOSIT = 20.5;

    final double WRIST_FOLDED_IN = 0.8333;
    final double WRIST_FOLDED_OUT = 0.5;

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        motorFrontLeft = hardwareMap.get(DcMotor.class, "Left_front");
        motorFrontRight = hardwareMap.get(DcMotor.class, "Right_front");
        motorBackLeft = hardwareMap.get(DcMotor.class, "Left_rear");
        motorBackRight = hardwareMap.get(DcMotor.class, "Right_rear");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        viperMotor = hardwareMap.get(DcMotor.class, "viperMotor");
        intake = hardwareMap.get(Servo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        // Set motor directions
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set arm motor to use encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize wrist in folded position
        wrist.setPosition(WRIST_FOLDED_IN);

        // Wait for the game driver to press play
        waitForStart();

        while (opModeIsActive()) {
            // Handle motor control for drive
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;
            double rightStickX = gamepad1.right_stick_x;

            // Calculate and set wheel powers
            double frontLeftPower = leftStickY + leftStickX;
            double frontRightPower = leftStickY - leftStickX;
            double backLeftPower = leftStickY + rightStickX;
            double backRightPower = leftStickY - rightStickX;

            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

            // Handle arm position and control
            updateArmControl();



            // Send telemetry
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void updateArmControl() {
        if (gamepad2.right_bumper) {
            armPosition = ARM_COLLECT;
            wrist.setPosition(WRIST_FOLDED_OUT);
            intake.setPosition(INTAKE_COLLECT);
        } else if (gamepad2.left_bumper) {
            armPosition = ARM_CLEAR_BARRIER;
        } else if (gamepad2.y) {            
            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
        } else if (gamepad2.dpad_left) {
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            wrist.setPosition(WRIST_FOLDED_IN);
            intake.setPosition(INTAKE_OFF);
//        } else if (gamepad2.left_bumper) {
//            armPosition = ARM_SCORE_SPECIMEN;
//            wrist.setPosition(WRIST_FOLDED_IN);
        } else if (gamepad2.dpad_up) {
            armPosition = ARM_ATTACH_HANGING_HOOK;
            intake.setPosition(INTAKE_OFF);
        } else if (gamepad2.dpad_down) {
            armPosition = ARM_WINCH_ROBOT;
            intake.setPosition(INTAKE_OFF);

        } else if (gamepad2.left_stick_button) {
            viperMotor.setPower(FINAL_EXTENDED_POSITION);
        } else if (gamepad2.right_stick_button) {
            viperMotor.setPower(-25);
            {


                // Fudge factor for arm control
                armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));
                armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
                armMotor.setPower(0.1);
            }
        }
    }
}

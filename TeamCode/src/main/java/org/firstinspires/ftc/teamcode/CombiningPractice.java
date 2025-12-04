package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Monkey", group = "Linear Opmode")
public class CombiningPractice extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor arm;

    Servo claw;


    @Override
    public void runOpMode() {
        // Hardware mapping
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        arm = hardwareMap.dcMotor.get("arm");



        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and set modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setTargetPosition(0);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //claw.setPosition(0);
        //claw.setDirection(Servo.Direction.REVERSE);
        arm.setPower(0.35);   // slow by default
        // normal speed

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Driving control
            double y = gamepad1.left_stick_y;//check
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(-frontRightPower);
            rightRear.setPower(-backRightPower);

            // Slide control (normal speed)

            // Arm control (slow precision mode)

            // Claw toggle
            if (gamepad1.right_bumper) {
                claw.setPosition(0.5);
            } else if (gamepad1.left_bumper) {
                claw.setPosition(0.4);
            }



        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OneControllerOpMode extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor arm;
    private DcMotor slide;
    private Servo claw;

    private boolean clawOpen = true;
    private boolean lastAState = false;

    int armTarget = 0;
    int slideTarget = 0;

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        arm = hardwareMap.dcMotor.get("arm");
        slide = hardwareMap.dcMotor.get("slide");
        claw = hardwareMap.servo.get("claw");

        // Set directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setTargetPosition(0);
        slide.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm.setPower(0.5);
        slide.setPower(0.5);

        // Wait for start
        waitForStart();

        while (opModeIsActive()) {
            // Driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // Slide control (encoder logic)
            if (gamepad1.right_trigger > 0.1) {
                slideTarget += 10;
            } else if (gamepad1.left_trigger > 0.1) {
                slideTarget -= 10;
            }
            slide.setTargetPosition(slideTarget);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(0.8);

            // Arm control (encoder logic)
            if (gamepad1.right_bumper) {
                armTarget += 10;
            } else if (gamepad1.left_bumper) {
                armTarget -= 10;
            }
            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.8);

            // Claw toggle
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAState) {
                clawOpen = !clawOpen;
                claw.setPosition(clawOpen ? 1.0 : 0.0); // Adjust positions as needed
            }
            lastAState = aPressed;

            // Optional: Telemetry
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("Slide Pos", slide.getCurrentPosition());
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Monkey", group = "Linear Opmode")
public class IgnitePeoria extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor arm;

    private Servo claw;

    private boolean clawOpen = true;
    private boolean lastAState = false;
    int defaultViperPosition = 0;
    boolean isHoldingTriggerR = false;
    boolean isHoldingPositionR = false;
    boolean isHoldingTriggerL = false;
    boolean isHoldingPositionL = false;
    int holdPosition = 0;

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

        claw = hardwareMap.servo.get("claw");

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD;
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset and set modes
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setTargetPosition(0);


        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.resetDeviceConfigurationForOpMode();
        claw.setPosition(0);
        claw.setDirection(Servo.Direction.REVERSE);
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
            if (gamepad1.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                isHoldingPositionR = false;

                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(0.2);
            }


            else if (isHoldingTriggerR) {
                isHoldingTriggerR= false;
                isHoldingPositionR = true;

                holdPosition = arm.getCurrentPosition();
                arm.setTargetPosition(holdPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ViperMotor.setPower(0.3);
            }

            if (gamepad1.left_trigger > 0.2) {
                isHoldingTriggerL = true;
                isHoldingPositionL = false;

                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-0.2);
            }


            else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                isHoldingPositionL = true;

                holdPosition = arm.getCurrentPosition();
                arm.setTargetPosition(holdPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ViperMotor.setPower(0.3);
            }
            // Claw toggle
            if (gamepad1.a) {
                claw.setPosition(1); // Adjust positions as needed
            } else if (gamepad1.b) {
                claw.setPosition(-1);

            }


            // Telemetry
            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("Claw", clawOpen ? "Open" : "Closed");
            telemetry.update();
        }
    }
}

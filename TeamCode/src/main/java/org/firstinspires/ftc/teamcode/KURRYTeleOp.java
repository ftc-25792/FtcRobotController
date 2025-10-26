package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "KURRY", group = "Linear Opmode")
public class KURRYTeleOP extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor launcherLeft, launcherRight, intake;
    private boolean isHoldingTriggerR = false;
    private boolean isHoldingPositionR = false;
    private boolean isHoldingTriggerL = false;
    private boolean isHoldingPositionL = false;
    private int holdPosition = 0;


    private double launchPower = 0.5; // adjustable shared power

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        launcherLeft = hardwareMap.get(DcMotor.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotor.class, "launcherRight");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // --- Motor Directions ---
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        launcherLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Intake Setup ---
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized — KURRyTeleOp");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


            if (gamepad2.y) {
                launchPower += 0.05;
                if (launchPower > 1.0) {
                    launchPower = 1.0;
                }
                sleep(200);
            } else if (gamepad2.x) {
                launchPower -= 0.05;
                if (launchPower < 0.1) {
                    launchPower = 0.1;
                }
                sleep(200);
            } else if (gamepad2.left_bumper){
                launchPower = 0.67;
            }

            if (gamepad2.a) {
                launcherLeft.setPower(launchPower);
            } else {
                launcherLeft.setPower(0);
            }
            if (gamepad2.right_bumper){
                launcherLeft.setPower(-launchPower);
            } else if(gamepad2.dpad_left){
                launcherRight.setPower(-launchPower);
            }


            if (gamepad2.b) {
                launcherRight.setPower(launchPower);
            } else {
                launcherRight.setPower(0);
            }
            if (gamepad2.dpad_down) {
                launcherLeft.setPower(0);
                launcherRight.setPower(0);
            }
            if (gamepad2.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                isHoldingPositionR = false;

                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(0.8); // intake forward
            } else if (isHoldingTriggerR) {
                isHoldingTriggerR = false;
                isHoldingPositionR = true;

                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.3); // hold position
            }

            if (gamepad2.left_trigger > 0.2) {
                isHoldingTriggerL = true;
                isHoldingPositionL = false;

                intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intake.setPower(-0.8); // reverse
            } else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                isHoldingPositionL = true;

                holdPosition = intake.getCurrentPosition();
                intake.setTargetPosition(holdPosition);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.8); // hold position
            }
            telemetry.addData("Launch Power", "%.2f", launchPower);
            telemetry.addData("D-Pad Down", "Stops both launchers");
            telemetry.update();
        }
    }
}

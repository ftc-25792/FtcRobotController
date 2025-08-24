package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ShrenikIsNotVeryHandsome", group = "Linear Opmode")
public class ViperMoveWithEncoder extends LinearOpMode {
    // Declare motors
    DcMotor ViperMotor;

    // Declare control variables
    int defaultViperPosition = 0;
    boolean isHoldingTriggerR = false;
    boolean isHoldingPositionR = false;
    boolean isHoldingTriggerL = false;
    boolean isHoldingPositionL = false;
    int holdPosition = 0;

    @Override
    public void runOpMode() {
        // Initialize motor
        ViperMotor = hardwareMap.get(DcMotor.class, "ViperMotor");

        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setDirection(DcMotor.Direction.REVERSE);
        // Store starting position for return
        defaultViperPosition = ViperMotor.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.2) {
                isHoldingTriggerR = true;
                isHoldingPositionR = false;

                ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ViperMotor.setPower(0.6);
            }


            else if (isHoldingTriggerR) {
                isHoldingTriggerR= false;
                isHoldingPositionR = true;

                holdPosition = ViperMotor.getCurrentPosition();
                ViperMotor.setTargetPosition(holdPosition);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ViperMotor.setPower(0.3);
            }

            if (gamepad1.left_trigger > 0.2) {
                isHoldingTriggerL = true;
                isHoldingPositionL = false;

                ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                ViperMotor.setPower(-0.6);
            }


            else if (isHoldingTriggerL) {
                isHoldingTriggerL = false;
                isHoldingPositionL = true;

                holdPosition = ViperMotor.getCurrentPosition();
                ViperMotor.setTargetPosition(holdPosition);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //ViperMotor.setPower(0.3);
            }


            // Telemetry to debug
            telemetry.addData("Viper Position", ViperMotor.getCurrentPosition());
            telemetry.addData("Target Position", ViperMotor.getTargetPosition());
            telemetry.update();
        }

    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Viper Slide Encoder Control")
public class EncoderViper extends LinearOpMode {

    private DcMotorEx viper;

    final double viperPower = 1.0;
    final double ARM_TICKS_PER_DEGREE = 1425.1 / 360.0;
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_SCORE_IN_HIGH = 46 * ARM_TICKS_PER_DEGREE;
    final double ARM_HANGING = -15 * ARM_TICKS_PER_DEGREE;

    double armPosition = ARM_COLLAPSED_INTO_ROBOT;

    @Override
    public void runOpMode() {


        viper = hardwareMap.get(DcMotorEx.class, "viper");


        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Viper Slide Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.dpad_up) {
                viper.setPower(viperPower);
            } else if (gamepad1.dpad_down) {
                viper.setPower(-viperPower);
            } else {
                viper.setPower(0);
            }


            if (gamepad1.a) {  // collapsed
                moveToPosition(ARM_COLLAPSED_INTO_ROBOT);
            } else if (gamepad1.b) {  // high scoring
                moveToPosition(ARM_SCORE_IN_HIGH);
            } else if (gamepad1.y) {  // hanging
                moveToPosition(ARM_HANGING);
            }

            telemetry.addData("Target Pos", armPosition);
            telemetry.addData("Current Pos", viper.getCurrentPosition());
            telemetry.addData("Power", viper.getPower());
            telemetry.update();
        }
    }

    private void moveToPosition(double targetTicks) {
        viper.setTargetPosition((int) targetTicks);
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viper.setPower(viperPower);

        // Wait until motor reaches target OR timeout
        while (opModeIsActive() && viper.isBusy()) {
            telemetry.addData("Moving to", targetTicks);
            telemetry.addData("Current", viper.getCurrentPosition());
            telemetry.update();
        }

        // Pause for 1 second after reaching position
        sleep(1000);

        // Stop motor after reaching
        viper.setPower(0);
    }
}

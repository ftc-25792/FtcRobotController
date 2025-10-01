package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Spinny Boi", group="Test")
public class Motor extends LinearOpMode {

    private DcMotor motor = null;
    @Override

    public void runOpMode() {

     motor = hardwareMap.get(DcMotor.class, "Motor");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {

            if (gamepad1.a) {
                motor.setPower(1);
            }

            if(gamepad1.right_trigger>0.5) {
                motor.setPower(-1);
                }

            if(gamepad1.left_stick_button);{
                motor.setPower(0);
            }
        }

    }
}

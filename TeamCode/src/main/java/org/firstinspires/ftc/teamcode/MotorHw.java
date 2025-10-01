package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorHw", group = "Linear Opmode")
public class MotorHw extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                motor.setPower(1);
                telemetry.addData("Motor Status", "forward");
            } else if (gamepad1.b) {
                motor.setPower(-1);
                telemetry.addData("motor status", "backward");
            } else{
                motor.setPower(0);
                telemetry.addData("motor status", "stopped");
            }
            telemetry.update();
            }

        }
    }




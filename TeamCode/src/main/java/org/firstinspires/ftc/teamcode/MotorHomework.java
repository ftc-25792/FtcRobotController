package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MotorHomework", group = "Autonomous")
public class MotorHomework extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(0.5);
            sleep(600);
            motor.setPower(0);
            motor.setPower(-0.5);
            sleep(550);
            motor.setPower(0);
        }
    }
}
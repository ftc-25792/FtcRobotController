package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "One Motor Velocity Test", group = "Auto")
public class PIDLF extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motor.setVelocityPIDFCoefficients(
                0.012,
                0.0,
                0.0004,
                14.0
        );

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {


            double targetVelocity = 2240;

            motor.setVelocity(targetVelocity);


            sleep(2000);


            motor.setVelocity(0);
        }
    }
}

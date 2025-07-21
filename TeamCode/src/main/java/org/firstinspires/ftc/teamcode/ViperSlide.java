package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Viper Slide", group = "Linear OpMode")
public class ViperSlide extends LinearOpMode {
    //Declare our motor
    private DcMotor viperSlideMotor = null;

    @Override
    public void runOpMode() {
        viperSlideMotor = hardwareMap.get(DcMotor.class, "viper_drive");
        viperSlideMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            viperSlideMotor.setPower(0.5);
            sleep(2000);
            viperSlideMotor.setPower(0);
            sleep(500);
            viperSlideMotor.setPower(-0.5);
            sleep(1000);
            viperSlideMotor.setPower(0);
        }
    }
}
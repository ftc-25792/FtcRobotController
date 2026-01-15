package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "--Single Motor Encoders", group = "Linear Opmode")

public class SingleMotorEncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws
            InterruptedException{
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);


        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        frontLeft.setPower(0.2);
        frontRight.setPower(0.2);
        while(opModeIsActive())
        {


            telemetry.addData("Front Left Motor", frontLeft.getConnectionInfo());
            telemetry.addData("Front Right Motor", frontRight.getConnectionInfo());
            telemetry.addData("Front Left Encoder Position", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder Pos", frontRight.getCurrentPosition());
            telemetry.update();

            sleep(200);
        }

    }
}

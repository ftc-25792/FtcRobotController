package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="MoveViper", group="TeleOp")
public class SimbaTeleOp extends LinearOpMode {

     DcMotor ViperMotor;


     int defaultViperPosition = 0;
     Boolean isHoldingTrigger = false;
     int holdPosition = 0;
     boolean isHoldingPosition = false;
    @Override
    public void runOpMode() {
        // Initialize motors
       ViperMotor = hardwareMap.get(DcMotor.class,"MoveViper");
       ViperMotor.setmode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       ViperMotor.setmode(DcMotor.RunMode.RUN_USING_ENCODER);
       ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       DefaultViperPosition = ViperMotor.getCurrentPosition();
       waitForStart();
       while (opModeIsActive()){
           if (gamepad1.right_trigger >0.2){
               isHoldingTrigger = true;
               isHoldingPosition = false;
               ViperMotor.setmode(DcMotor.RunMode.RUN_USING_ENCODER);
               ViperMotor.setPower(0.6);
           }

           else if (isHoldingTrigger) {
               isHoldingTrigger = false;
               isHoldingPosition = true;

               holdPosition = ViperMotor.getCurrentPosition();
               ViperMotor.settargetPostion(holdPosition);
               ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }

           if(gamepad1.a) {
               ViperMotor.setTargetPosition(defaultViperPosition);
               ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               ViperMotor.setPower(1.0);
           }

           telementry.addData("Viper Position", ViperMotor.getCurrentPosition());
           telementry.addData("Target Position", ViperMotor.getCurrentPosition());
           telementry.update();

       }

}

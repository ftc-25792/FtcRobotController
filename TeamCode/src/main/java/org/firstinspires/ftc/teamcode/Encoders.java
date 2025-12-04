package org.firstinspires.ftc.teamcode;
//This is where you imports will go
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Encoders extends LinearOpMode {
    //This is where you will define things
    DcMotor myMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        //This is where you will initialize the things you defined

        // Setting Up Hardware Map
        myMotor = hardwareMap.get(DcMotor.class, "myMotor");

        //Resetting Encoders Back to Zero Ticks
        myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        myMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        waitForStart();

        while (opModeIsActive()) {
            //This is where your main code will go

            double position = myMotor.getCurrentPosition();

            telemetry.addData("Encoder Position", position);
            telemetry.update();






            }

    }
}
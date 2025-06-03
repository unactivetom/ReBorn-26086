package org.firstinspires.ftc.teamcode.PremierEvent.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


//
@Autonomous(name="MainAuto", group="PremierEvent")
public class MainAuto extends LinearOpMode {
    Drive2 drive = new Drive2();

    public void Write(String text, Object value){
        telemetry.addData(text, value);
    }
    
    @Override
    public void runOpMode() {
        //HIER CODE IN
        drive.DriveInit(hardwareMap);
        telemetry.addData("Init", 1);
        telemetry.update();
        String position = drive.DriveToPos(new SparkFunOTOS.Pose2D(0.4, 0.2, 0), 0.1);
        telemetry.addData("position:", position);
        telemetry.addData("On pos", 1);
        telemetry.update();
        //drive.Turn(30);


    }

}

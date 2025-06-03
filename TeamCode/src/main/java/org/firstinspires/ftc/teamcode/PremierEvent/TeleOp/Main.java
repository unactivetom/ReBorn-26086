package org.firstinspires.ftc.teamcode.PremierEvent.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//
@TeleOp(name="Main", group="PremierEvent")
public class Main extends OpMode {

    Drive drive = new Drive();
    Lift lift = new Lift();

    @Override
    public void init() {

        drive.DriveInit(hardwareMap);
        lift.InitLift(hardwareMap);


        telemetry.addData("Init", 0);
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.DriveLoop(gamepad1);
        lift.manualControl(gamepad2);

        telemetry.addData("Loop", 0);
        telemetry.update();
    }

    public void Write(String text, Object value){
        telemetry.addData(text, value);
    }
}

package org.firstinspires.ftc.teamcode.PremierEvent.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

//

@TeleOp(name="DriveBaseOnly", group="PremierEvent")
public class DriveBaseOnly extends OpMode {

    Drive drive = new Drive();

    @Override
    public void init() {

        drive.DriveInit(hardwareMap);

        telemetry.addData("Init", 0);
        telemetry.update();

    }

    @Override
    public void loop() {



        telemetry.addData("Loop", 0);
        telemetry.update();

    }
}

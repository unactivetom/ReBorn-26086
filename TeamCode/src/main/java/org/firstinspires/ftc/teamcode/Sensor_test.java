package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@TeleOp(name  = "sensors", group = "Linear OpMode")
public class Sensor_test extends LinearOpMode {

    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;




    @Override
    public void runOpMode() throws InterruptedException {

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");


        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("distanceSensor: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("colorSensor: ", colorSensor.argb());
            telemetry.addData("touchSensor: ", touchSensor.isPressed());
            telemetry.update();
        }

    }
}

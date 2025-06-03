package org.firstinspires.ftc.teamcode.backup4.Driver2;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorOld {
    private DistanceSensor distanceSensor;

    public void Sensor_setup(HardwareMap hardwareMap){

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

    }
    public double Distance_sensor_out(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}

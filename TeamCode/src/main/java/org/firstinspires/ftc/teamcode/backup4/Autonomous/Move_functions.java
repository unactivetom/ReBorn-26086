package org.firstinspires.ftc.teamcode.backup4.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.backup4.Constant_values;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Move_functions {

    private DcMotorEx left_drive;
    private DcMotorEx right_drive;
    IMU imu;
    private IMU.Parameters IMU_parameters;
    public void Drive_setup(HardwareMap hardwareMap){
        //Let the program know what motor is left, and what motor is right
        left_drive  = hardwareMap.get(DcMotorEx.class, "drive_left");
        right_drive = hardwareMap.get(DcMotorEx.class, "drive_right");

        //We also declare what direction they are turning
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU_parameters = new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES, 90, 0, -45, 0)));
        imu.initialize(IMU_parameters);

    }

    public void Move(int distance, double speed){
        left_drive.setTargetPosition(left_drive.getCurrentPosition() + (distance*Constant_values.TICKS_PER_CM) - 1);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() + (distance*Constant_values.TICKS_PER_CM) - 1);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_drive.setVelocity(speed); //should be pretty high
        right_drive.setVelocity(speed);

        left_drive.setPower(1);
        right_drive.setPower(1);
    }

    public void Turn(int degrees){
        imu.resetYaw();
        if(degrees > 180){
            degrees = -(360 - degrees);
        }

        if(degrees > 0) {
            while (degrees > -imu.getRobotYawPitchRollAngles().getYaw()) {
                left_drive.setVelocity(200);
                right_drive.setVelocity(-200);
                left_drive.setPower(0.8);
                right_drive.setPower(0.8);
            }
        }
        if(degrees < 0) {
            while (degrees < -imu.getRobotYawPitchRollAngles().getYaw()) {
                left_drive.setVelocity(-200);
                right_drive.setVelocity(200);
                left_drive.setPower(0.8);
                right_drive.setPower(0.8);
            }
        }


    }

}

package org.firstinspires.ftc.teamcode.backup4.Driver2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.backup4.Constant_values;

public class DriveOld {

    //Define DC motors, these are the motors for driving
    private DcMotorEx left_drive;
    private DcMotorEx right_drive;

    //These variables are just to calculate the speed
    public double left_power;
    public double right_power;
    public double yaw = 0;

    //For gyro




    public void Drive_setup(HardwareMap hardwareMap){
        //Let the program know what motor is left, and what motor is right
        left_drive  = hardwareMap.get(DcMotorEx.class, "drive_left");
        right_drive = hardwareMap.get(DcMotorEx.class, "drive_right");

        //We also declare what direction they are turning
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void Move(Gamepad gamepad1) {
        double slow_drive = -gamepad1.left_stick_y * Constant_values.SLOW_SPEED;
        double slow_turn = gamepad1.left_stick_x * Constant_values.SLOW_SPEED;
        double fast_turn  =  gamepad1.right_stick_x;
        slow_drive += Fast_drive(gamepad1); //We want the fast drive to overwrite the slow speed

        if(gamepad1.cross){
            Brake(gamepad1);
        }else {
             //Make the variable stay between the boundaries the motor can handle
            left_power = Range.clip((slow_drive + slow_turn + fast_turn), -1.0, 1.0);
            right_power = Range.clip((slow_drive - slow_turn - fast_turn), -1.0, 1.0);

             //Send the power to the motors
            left_drive.setPower(left_power);
            right_drive.setPower(right_power);
        }
    }

    private double Fast_drive(Gamepad gamepad1){ //this function return a value of 1 when pressed, and a value of 0 when not
        double drive_speed = 0; //every frame reset to 0
        if(gamepad1.right_trigger > 0){ //if the right trigger is pressed

            drive_speed++; //Makes the Motors go forward

        }else if(gamepad1.left_trigger > 0){ //if the left trigger is p

            drive_speed--; //Makes the Motors go backwards

        }

        return(drive_speed);
    }


    private void Brake(Gamepad gamepad1) {
        //Sets the motor on full trust backwards
        while(gamepad1.cross){
            left_drive.setPower(-left_power);
            right_drive.setPower(-right_power);
        }

    }
}

package org.firstinspires.ftc.teamcode.backup4.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.backup4.Constant_values;

public class Arm_functions {

    private Servo gripper;
    private Servo wrist;
    public DcMotorEx arm_left;
    private DcMotorEx arm_right;
    public void Arm_setup(HardwareMap hardwareMap2){
        gripper = hardwareMap2.get(Servo.class, "gripper");
        wrist = hardwareMap2.get(Servo.class, "servo_1");
        arm_left = hardwareMap2.get(DcMotorEx.class, "arm_left");
        arm_right = hardwareMap2.get(DcMotorEx.class, "arm_right");

        gripper.setPosition(Constant_values.GRIPPER_POSITION_OPEN);
        wrist.setPosition(Constant_values.WRIST_POSITION_INIT);

        //direction of the motors
        arm_left.setDirection(DcMotor.Direction.FORWARD);
        arm_right.setDirection(DcMotor.Direction.REVERSE);

        arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Make the motor brake when doing nothing
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void Setmode(Bot_state state){
        switch (state){
            case INIT:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_OPEN);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_INIT);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_INIT);
                wrist.setPosition(Constant_values.WRIST_POSITION_INIT);
                break;
            case MANUAL:
                //Used for overwrite
                break;
            case INTAKE:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_OPEN);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_INTAKE);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_INTAKE);
                wrist.setPosition(Constant_values.WRIST_POSITION_INTAKE);
                break;
            case LOW_CLIP:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_CLOSED);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_LOW_CLIP);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_LOW_CLIP);
                wrist.setPosition(Constant_values.WRIST_POSITION_LOW_CLIP);
                break;
            case HIGH_CLIP:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_CLOSED);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_HIGH_CLIP);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_HIGH_CLIP);
                wrist.setPosition(Constant_values.WRIST_POSITION_HIGH_CLIP);
                break;
            case LOW_BASKET:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_CLOSED);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_LOW_BASKET);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_LOW_BASKET);
                wrist.setPosition(Constant_values.WRIST_POSITION_LOW_BASKET);
                break;
            case HIGH_BASKET:
                gripper.setPosition(Constant_values.GRIPPER_POSITION_CLOSED);
                arm_left.setTargetPosition(Constant_values.ARM_POSITION_HIGH_BASKET);
                arm_right.setTargetPosition(Constant_values.ARM_POSITION_HIGH_BASKET);
                wrist.setPosition(Constant_values.WRIST_POSITION_HIGH_BASKET);
                break;
        }
        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_left.setVelocity(Constant_values.ARM_SPEED);
        arm_right.setVelocity(Constant_values.ARM_SPEED);
        arm_left.setPower(1.0);
        arm_right.setPower(1.0);

        while(arm_left.isBusy()){}
    }
    public void Gripper_overwrite(boolean is_open){
        if(is_open){
            gripper.setPosition(Constant_values.GRIPPER_POSITION_OPEN);
        }else{
            gripper.setPosition(Constant_values.GRIPPER_POSITION_CLOSED);
        }
    }

    public void Arm_overwrite(int arm_position, double wrist_position){
        Setmode(Bot_state.MANUAL);
        arm_left.setTargetPosition(arm_position);
        arm_right.setTargetPosition(arm_position);

        arm_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_left.setVelocity(Constant_values.ARM_SPEED);
        arm_right.setVelocity(Constant_values.ARM_SPEED);

        wrist.setPosition(wrist_position);

        arm_left.setPower(1.0);
        arm_right.setPower(1.0);
    }



}

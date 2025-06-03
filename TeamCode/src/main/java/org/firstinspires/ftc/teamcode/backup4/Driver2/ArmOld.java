package org.firstinspires.ftc.teamcode.backup4.Driver2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.backup4.Constant_values;

public class ArmOld {
    private Servo gripper;
    public Servo wrist;
    public DcMotorEx arm_left;
    public DcMotorEx arm_right;
    double gripper_position = Constant_values.GRIPPER_POSITION_CLOSED; //Moet ergens anders heen
    double wrist_position = Constant_values.WRIST_POSITION_INIT; //Moet ergens anders heen
    public double arm_position = Constant_values.ARM_POSITION_INIT;


    public void Arm_setup(HardwareMap hardwareMap2) {
        gripper = hardwareMap2.get(Servo.class, "gripper");
        wrist = hardwareMap2.get(Servo.class, "servo_1");
        arm_left = hardwareMap2.get(DcMotorEx.class, "arm_left");
        arm_right = hardwareMap2.get(DcMotorEx.class, "arm_right");

        gripper.setPosition(Constant_values.GRIPPER_POSITION_OPEN);
        //direction of the motors
        arm_left.setDirection(DcMotor.Direction.FORWARD);
        arm_right.setDirection(DcMotor.Direction.REVERSE);

        arm_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Make the motor brake when doing nothing
        arm_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void Gripper(Gamepad gamepad2) {


        if (gamepad2.right_bumper) {
            gripper_position = Constant_values.GRIPPER_POSITION_CLOSED;
        }
        if (gamepad2.left_bumper) {
            gripper_position = Constant_values.GRIPPER_POSITION_OPEN;
        }

        gripper.setPosition(gripper_position);
    }

    public void Servo_arm(Gamepad gamepad2) {


        if (gamepad2.left_stick_y != 0) {
            wrist_position = Range.clip(wrist_position + gamepad2.left_stick_y / 80, 0, 1);

            wrist.setPosition(wrist_position);
        }



    }

    public void Motor_arm(Gamepad gamepad2) {
        if (gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_y <= -0.1) {


            arm_left.setVelocity(-gamepad2.right_stick_y * 180);
            arm_right.setVelocity(-gamepad2.right_stick_y * 180);
            arm_left.setPower(1.0);
            arm_right.setPower(1.0);


        } else if (arm_right.getVelocity() != 0 || arm_right.getVelocity() != 0) {

            arm_left.setVelocity(0);
            arm_right.setVelocity(0);
        }
    }
    
}



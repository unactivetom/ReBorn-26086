package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field Centric DriveOld", group="TeleOp")
public class FieldCentricDrive extends OpMode {



    private BNO055IMU imu;
    private DcMotor linksachter;
    private DcMotor linksvoor;
    private DcMotor rechtsachter;
    private DcMotor rechtsvoor;

    
    @Override
    public void init() {

        //color_sensor = hardwareMap.get(ColorSensor.class, "color sensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        linksachter = hardwareMap.get(DcMotor.class, "LeftBack");
        linksvoor = hardwareMap.get(DcMotor.class, "LeftFront");
        rechtsachter = hardwareMap.get(DcMotor.class, "RightBack");
        rechtsvoor = hardwareMap.get(DcMotor.class, "RightFront");


        // Set motor directions if needed
        rechtsvoor.setDirection(DcMotorSimple.Direction.REVERSE);
        rechtsachter.setDirection(DcMotorSimple.Direction.REVERSE);


        // IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    @Override
    public void loop() {


        Drive();

        telemetry.update();
    }

    void Drive(){
        // Get IMU heading in radians
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        double heading = angles.firstAngle;

        // Get joystick inputs for movement
        double y = -gamepad1.left_stick_y; // forward/back
        double x = -gamepad1.left_stick_x; // strafe, with slight adjustment
        double turn = -gamepad1.right_stick_x; // rotation



        // Rotate the input coordinates based on the field-centric angle
        double rotatedX = x * Math.cos(-heading) + y * Math.sin(-heading);
        double rotatedY = -x * Math.sin(-heading) + y * Math.cos(-heading);

        // Calculate motor powers for mecanum drive
        double leftFrontPower = rotatedY + rotatedX + turn;
        double rightFrontPower = rotatedY - rotatedX - turn;
        double leftBackPower = rotatedY - rotatedX + turn;
        double rightBackPower = rotatedY + rotatedX - turn;

        // Normalize motor powers if any exceeds 1.0
        double max = Math.max(1.0, Math.abs(leftFrontPower));
        max = Math.max(max, Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        leftFrontPower /= max;
        rightFrontPower /= max;
        leftBackPower /= max;
        rightBackPower /= max;

        // Set motor powers
        linksvoor.setPower(leftFrontPower);
        rechtsvoor.setPower(rightFrontPower);
        linksachter.setPower(leftBackPower);
        rechtsachter.setPower(rightBackPower);


        telemetry.addData("Heading (rad)", heading);
        telemetry.addData("LF Power", leftFrontPower);
        telemetry.addData("RF Power", rightFrontPower);
        telemetry.addData("LB Power", leftBackPower);
        telemetry.addData("RB Power", rightBackPower);
    }



}
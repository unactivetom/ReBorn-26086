package org.firstinspires.ftc.teamcode.PremierEvent.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//
public class Drive {

    private BNO055IMU imu;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private Main main = new Main();

    void DriveInit(HardwareMap hardwareMap){

        //color_sensor = hardwareMap.get(ColorSensor.class, "color sensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");


        // Set motor directions if needed
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);


        // IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }
    void DriveLoop(Gamepad gamepad1){
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
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);


        main.Write("leftFront", leftFrontPower);
        main.Write("rightFront", rightFrontPower);
        main.Write("leftBack", leftBackPower);
        main.Write("rightBack", rightBackPower);

    }
}

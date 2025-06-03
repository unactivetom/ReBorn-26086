package org.firstinspires.ftc.teamcode.PremierEvent.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.reflect.Array;
import java.util.ArrayList;

//

public class Drive {

    private BNO055IMU imu;
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    private SparkFunOTOS myOtos;



    void DriveInit(HardwareMap hardwareMap){

        //color_sensor = hardwareMap.get(ColorSensor.class, "color sensor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

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

        // configures the otos sensor.
        configureOtos();

    }

    public String DriveToPos(SparkFunOTOS.Pose2D pos, double speed){

        double x, y;

        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();

        while((pos.x - currentPos.x) > 0.01 || (pos.x - currentPos.x) < -0.01 || (pos.y - currentPos.y) > 0.01 || (pos.y - currentPos.y) < -0.01) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle;
            currentPos = myOtos.getPosition();



            if(currentPos.x-pos.x > 0){x = speed;
            } else if (currentPos.x-pos.x < 0) {x = -speed;
            } else {x = 0;}

            if(currentPos.y-pos.y < 0){y = speed;
            } else if (currentPos.y-pos.y > 0) {y = -speed;
            } else {y = 0;}

            // Rotate the input coordinates based on the field-centric angle
            double rotatedX = x * Math.cos(-heading) + y * Math.sin(-heading);
            double rotatedY = -x * Math.sin(-heading) + y * Math.cos(-heading);

            // Calculate motor powers for mecanum drive
            double leftFrontPower = rotatedY + rotatedX;
            double rightFrontPower = rotatedY - rotatedX;
            double leftBackPower = rotatedY - rotatedX;
            double rightBackPower = rotatedY + rotatedX;

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
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        return(currentPos.toString());
    }

    public void Turn(int degree){
        SparkFunOTOS.Pose2D currentPos = myOtos.getPosition();
        double heading = currentPos.h;
        double turnTo = heading-degree;

        while(turnTo < 3 && turnTo > -3){
            if(turnTo > 0){
                leftFront.setPower(0.3);
                rightFront.setPower(-0.3);
                leftBack.setPower(0.3);
                rightBack.setPower(-0.3);
            }
            if(turnTo < 0){
                leftFront.setPower(-0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(-0.3);
                rightBack.setPower(0.3);
            }
            currentPos = myOtos.getPosition();
            heading = currentPos.h;
            turnTo = heading-degree;
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private void configureOtos() {

        //set units, normal meter and degrees, ###test if cm work###
        myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // offset from robot center
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.001);
        myOtos.setAngularScalar(1.0);

        // for use of Imu for calibrating, recommendated before every opmode
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // set the origin
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

    }


}

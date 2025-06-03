package org.firstinspires.ftc.teamcode.backup4.Driver2;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Base_2025", group="Linear OpMode")

public class Base_2025 extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    DriveOld drive = new DriveOld();
    ArmOld arm = new ArmOld();
    SensorOld sensor = new SensorOld();
    long count = 0;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        drive.Drive_setup(hardwareMap);
        arm.Arm_setup(hardwareMap);
        sensor.Sensor_setup(hardwareMap);

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {



            arm.Gripper(gamepad2);
            arm.Servo_arm(gamepad2);
            arm.Motor_arm(gamepad2);
            drive.Move(gamepad1);



            count++;

            telemetry
            .addData("left Y", gamepad2.left_stick_y)
            .addData("right Y", gamepad2.right_stick_y)
            .addData("arm position", arm.arm_left.getCurrentPosition())
            .addData("wrist position", arm.wrist.getPosition())
            .addData("count", count)
            .addData("arm velocity", arm.arm_left.getVelocity())
            .addData("yaw", drive.yaw)
            .addData("arm pos", arm.arm_position)
            .addData("wheel power left, right", drive.left_power + " " + drive.right_power)
            .addData("distance sensor: ", sensor.Distance_sensor_out())

            .addData("Status", "Run Time: " + time);
            telemetry.update();
        }
    }
}

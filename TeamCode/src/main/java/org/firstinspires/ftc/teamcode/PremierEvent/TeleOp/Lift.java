package org.firstinspires.ftc.teamcode.PremierEvent.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

//
public class Lift {

    private final int MAX_POSITION = 3000; // change based on your slide length
    private final int MIN_POSITION = 0;

    // Preset positions
    private final int GROUND_POS = 0;
    private final int LOW_POS = 1000;
    private final int MEDIUM_POS = 2000;
    private final int HIGH_POS = 3000;


    private DcMotor liftLeft;
    private DcMotor liftRight;
    Main main = new Main();

    void InitLift(HardwareMap hardwareMap){
        liftLeft = hardwareMap.get(DcMotor.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotor.class, "liftRight");

        liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        liftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void presetControl(Gamepad gamepad2) {
        if (gamepad2.a) {
            moveToPosition(GROUND_POS);
        } else if (gamepad2.x) {
            moveToPosition(LOW_POS);
        } else if (gamepad2.y) {
            moveToPosition(MEDIUM_POS);
        } else if (gamepad2.b) {
            moveToPosition(HIGH_POS);
        }
    }

    public void manualControl(Gamepad gamepad2) {
        double power = -gamepad2.left_stick_y; // Push up to extend, pull down to retract
        int currentPos = liftLeft.getCurrentPosition();

        // Soft lock check
        if ((power > 0 && currentPos < MAX_POSITION) || (power < 0 && currentPos > MIN_POSITION)) {
            liftLeft.setPower(power * 0.75); // scale to reduce strain
            liftRight.setPower(power * 0.75); // scale to reduce strain
        } else {
            liftLeft.setPower(0); // stop if beyond limit
            liftRight.setPower(0); // stop if beyond limit
        }

        main.Write("Slide Position: ", currentPos);
    }


    public void moveToPosition(int targetPosition) {
        liftLeft.setTargetPosition(targetPosition);
        liftRight.setTargetPosition(targetPosition);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(0.7); // Moderate power for smooth move
        liftRight.setPower(0.7);

        // Wait until slide reaches target (or timeout for safety)
        while (liftLeft.isBusy()) {
            main.Write("Moving to: ", targetPosition);
            main.Write("Current: ", liftLeft.getCurrentPosition());
        }

        liftLeft.setPower(0);
        liftRight.setPower(0);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}

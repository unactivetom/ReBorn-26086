package org.firstinspires.ftc.teamcode.backup4.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "base_autonomous")

public class Base_2025_autonomous extends LinearOpMode {
    Arm_functions arm = new Arm_functions();
    Move_functions move = new Move_functions();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        runtime.reset();
        waitForStart();
        arm.Arm_setup(hardwareMap);
        move.Drive_setup(hardwareMap);
        arm.Setmode(Bot_state.INIT);
        //VANAF HIER BOUWEN


        move.Move(130, 200);
        move.Turn(79);



        //TOT HIER BOUWEN
        arm.Setmode(Bot_state.INIT);
    }




}



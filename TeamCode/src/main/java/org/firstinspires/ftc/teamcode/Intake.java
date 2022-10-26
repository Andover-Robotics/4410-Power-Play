package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Bot {
    public static Bot instance;
    public static Bot getInstance(){
        if(instance != null) {
            return instance;
        }

    }

}



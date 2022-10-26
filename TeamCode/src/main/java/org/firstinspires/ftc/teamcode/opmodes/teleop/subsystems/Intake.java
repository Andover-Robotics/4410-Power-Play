package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;


public class Bot {

    public static Bot instance;

    public static Bot getInstance(){
        if(instance != null) {
            return instance;
        }

    }

}



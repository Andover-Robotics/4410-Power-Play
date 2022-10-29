package org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems;


public class Bot {

    public static Bot instance;
    public org.firstinspires.ftc.teamcode.opmodes.teleop.subsystems.slide slide;

    public static Bot getInstance(){
        if(instance != null) {
            return instance;
        }

    }

}



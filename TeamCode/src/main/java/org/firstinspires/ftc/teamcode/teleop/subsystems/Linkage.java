package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Linkage {
    private final Servo linkLeft, linkRight, railLeft, railRight;

    public static double railOut = 1, railIn = 0, linkOut = 1, linkIn = 0;

    public Linkage(OpMode opMode){
        linkLeft = opMode.hardwareMap.servo.get("linkLeft");
        linkRight = opMode.hardwareMap.servo.get("linkRight");
        linkLeft.setDirection(Servo.Direction.FORWARD);
        linkRight.setDirection(Servo.Direction.FORWARD);
        railLeft = opMode.hardwareMap.servo.get("railLeft");
        railRight = opMode.hardwareMap.servo.get("railRight");
        railLeft.setDirection(Servo.Direction.FORWARD);
        railRight.setDirection(Servo.Direction.FORWARD);
    }

    private void setLinkage(double position){
        linkLeft.setPosition(position);
        linkRight.setPosition(1-position);
    }

    private void setRail(double position){
        railLeft.setPosition(position);
        railRight.setPosition(1-position);
    }

    public void fullOut(){
        setLinkage(linkOut);
        setRail(railOut);
    }

    public void intake(){
        setLinkage(linkIn);
        setRail(railOut);
    }

    public void outtake(){
        setLinkage(linkIn);
        setRail(railIn);
    }

}

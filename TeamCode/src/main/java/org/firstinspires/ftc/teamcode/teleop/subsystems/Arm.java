package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    private final Servo armLeft, armRight, bar;

    public static double armIntake = 0.25, armOuttake = 0.5, armStorage = 0.75, barIntake = 0.61, barOuttakeUp = 0.5, barOuttakeDown = 0.43;

    public Arm(OpMode opMode){
        armLeft = opMode.hardwareMap.servo.get("armLeft");
        armRight = opMode.hardwareMap.servo.get("armRight");
        armLeft.setDirection(Servo.Direction.FORWARD);
        armRight.setDirection(Servo.Direction.FORWARD);
        bar = opMode.hardwareMap.servo.get("bar");
        bar.setDirection(Servo.Direction.FORWARD);
    }

    private void setArm(double position){
        armLeft.setPosition(position);
        armRight.setPosition(1-position);
    }

    private void setBar(double position){
        bar.setPosition(position);
    }

    public void intake(){
        setBar(barIntake);
        setArm(armIntake);
    }

    public void storage(){
        setArm(armStorage);
        setBar(barOuttakeUp);
    }

    public void outtake(){
        setBar(barOuttakeUp);
        setArm(armOuttake);
    }

    public void secure() {
        setBar(barOuttakeDown);
        setArm(armOuttake);
    }
}

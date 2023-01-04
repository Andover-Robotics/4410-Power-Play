package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private final Servo left, right;

    public Claw(OpMode opMode){
        left = opMode.hardwareMap.servo.get("clawLeft");
        right = opMode.hardwareMap.servo.get("clawRight");
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        left.setPosition(0.40);
        right.setPosition(0.46);
    }

    public void close(){
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        left.setPosition(0.30);
        right.setPosition(0.56);
    }

}
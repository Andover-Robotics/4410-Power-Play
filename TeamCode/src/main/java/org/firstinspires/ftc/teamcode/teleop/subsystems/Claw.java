package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw {
    private final Servo left, right;
    private final DistanceSensor proxSensor;

    public Claw(OpMode opMode){
        left = opMode.hardwareMap.servo.get("clawLeft");
        right = opMode.hardwareMap.servo.get("clawRight");
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        proxSensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    public void open(){
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        left.setPosition(0.55);
        right.setPosition(0.29);
    }

    public void close(){
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        left.setPosition(0.30);
        right.setPosition(0.56);
    }

    public void openRight()
    {
        left.setPosition(0.4);
        right.setPosition(0.29);
    }

    public void openLeft()
    {
        left.setPosition(0.55);
        right.setPosition(0.44);
    }

    public double getDistance(){
        return proxSensor.getDistance(DistanceUnit.MM);
    }
}

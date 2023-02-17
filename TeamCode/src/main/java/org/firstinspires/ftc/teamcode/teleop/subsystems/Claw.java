package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Claw {
    private final Servo claw;
    //private final DistanceSensor proxSensor;
    public static double open = 1, close = 0;
  //  public static int proximityBound = 85;

    public Claw(OpMode opMode){
        claw = opMode.hardwareMap.servo.get("claw");
       // flip = opMode.hardwareMap.servo.get("flip");
        claw.setDirection(Servo.Direction.FORWARD);
      //  flip.setDirection(Servo.Direction.FORWARD);
        //proxSensor = opMode.hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    public void open(){
        claw.setPosition(open);
    }

    public void close(){
        claw.setPosition(close);
    }

   /* public double getDistance(){
        return proxSensor.getDistance(DistanceUnit.MM);
    }

    */
}

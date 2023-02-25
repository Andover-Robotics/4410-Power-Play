package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

import java.util.Map;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, cycleTime = 1;

    private boolean debugMode = false;
    private boolean cancelPrevAction = false;
    private boolean autoMode = false;
    private int index = 4;

    Thread armOuttake;

    private GamepadEx gp1, gp2;


    @Override
    public void runOpMode() throws InterruptedException {
        //TODO EXPERIMENTAL CODE ================= bad
//        Bot.instance = null;
//        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
//            entry.getValue().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            while (!isStopRequested() && Math.abs(entry.getValue().getCurrentPosition()) > 1) {
////                idle();
////            }
//        }
        //End experimental code ===================

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot.instance = null;
        bot = Bot.getInstance(this);

        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        bot.resetProfiler();

        bot.state = Bot.BotState.STORAGE;
        bot.arm.storage();
        bot.claw.open();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            gp1.readButtons();
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.BACK)){
                debugMode = !debugMode;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.BACK)){
                bot.fieldCentricRunMode = !bot.fieldCentricRunMode;
            }

            if(!debugMode) {//finite state
                if (bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B) || gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        bot.claw.close();
                    }
                    if(gp2.getButton(GamepadKeys.Button.X) || gp2.getButton(GamepadKeys.Button.Y)){
                        bot.claw.open();
                        cancelPrevAction = true;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.A) || gp2.wasJustReleased(GamepadKeys.Button.B)){
                        if(!cancelPrevAction){
                            bot.storage();
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER)){
                        if(!cancelPrevAction){
                            autoMode = true;
                            goToOuttakeLeft();
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
                        if(!cancelPrevAction){
                            autoMode = true;
                            goToOuttakeRight();
                        }
                        cancelPrevAction = false;
                    }
                } else if (bot.state == Bot.BotState.STORAGE) {
                    if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                        cancelPrevAction = true;
                        bot.intakeIn();
                    } else if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                        cancelPrevAction = true;
                        bot.intakeOut();
                    } else if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                        bot.outtake();
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                        bot.turret.runToAutoOuttakeLeft(bot.getIMU());
                    }else if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        bot.turret.runToAutoOuttakeRight(bot.getIMU());
                    }
//                    if(gp2.wasJustPressed(GamepadKeys.Button.B)){
//                        bot.state = Bot.BotState.OUTTAKE;
//                        autoMode = true;
//                        goToOuttake();
//                    }
                } else if (bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        bot.outtake();
                        cancelPrevAction = true;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                        cancelPrevAction = false;
                    }
                    if(gp2.getButton(GamepadKeys.Button.Y)){
                        if(!cancelPrevAction) {
                            bot.secure();
                        }
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.Y)){
                        if(!cancelPrevAction){
                            bot.claw.open();
                            bot.storage();
                            if(autoMode) {
                                armOuttake = new Thread(() -> {
                                    sleep(400);
                                    bot.turret.runToIntake(bot.getIMU());
                                });
                            }
                            autoMode = false;
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                        bot.storageNotDown();
                    }
                }
//                Vector2d stickVector = new Vector2d(gp2.getRightX(), gp2.getRightY()); // TODO get this working
//                double angle = stickVector.angle() * 180/Math.PI;
//                if(stickVector.norm() > 0.5) {
//                    bot.turret.runToAngle(angle, bot.getIMU());
//                }
                bot.turret.runManual(gp2.getLeftX());
                bot.horizSlides.runManual(-gp2.getLeftY());
//                bot.slides.runManual(-gp2.getLeftY());
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.slides.runToTopTeleOp();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.slides.runToMiddle();
                }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.slides.runToLow();
                }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.slides.runToBottom();
                }
            }else{//debug mode
                bot.horizSlides.runManual(-gp2.getRightY());
                bot.slides.runManual(-gp2.getLeftY());
                bot.turret.runManual(gp2.getLeftX());

                if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                    bot.resetEncoder();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                    Bot.instance = null;
                    bot = Bot.getInstance(this);
                    bot.slides.resetProfiler();
                    bot.horizSlides.resetProfiler();
                }


                if (gp2.getButton(GamepadKeys.Button.A)) {
                    bot.claw.open();
                } else {
                    bot.claw.close();
                }


                if (gp2.getButton(GamepadKeys.Button.X)) {
                    bot.arm.intakeAuto(index);
                } else if (gp2.getButton(GamepadKeys.Button.Y)) {
                    bot.arm.outtake();
                } else if (gp2.getButton(GamepadKeys.Button.B)) {
                    bot.arm.secure();
                } else {
                    bot.arm.storage();
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    index++;
                    if (index > 5) {
                        index = 5;
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    index--;
                    if (index < 0) {
                        index = 0;
                    }
                }
            }

            telemetry.addData("drive current", bot.getCurrent());
            telemetry.addData("slide current", bot.slides.getCurrent());
            telemetry.addData("getIMU", bot.getIMU());
            telemetry.addData("debug", debugMode);
            telemetry.addData("fieldCentric", bot.fieldCentricRunMode);
            telemetry.addData("state", bot.state.toString());
            telemetry.update();

            bot.slides.periodic();
            bot.turret.periodic();
            bot.horizSlides.periodic();
            drive();
        }
    }


    //copied from auto
    public static int driveTime = 2200, timeSlidesUp = 850, timeSlidesDown = 550, timeOuttake = 350, timeConeDrop = 150, timeIntakeDown = 200, timeIntakeOut = 700, timeIntakeClose = 150, timeIntakeUp = 500, timeIntakeIn = 400;//old 400


    private void goToOuttakeRight() {//TODO change values to use the stored values


        armOuttake = new Thread(() ->{
            bot.storage();
            bot.state = Bot.BotState.OUTTAKE;
            sleep(400);
            bot.turret.runToAutoOuttakeLeft(bot.getIMU());
            bot.slides.runToTopTeleOp();
        });
        armOuttake.start();

    }

    private void goToOuttakeLeft() {//TODO change values to use the stored values


        armOuttake = new Thread(() ->{
            bot.storage();
            bot.state = Bot.BotState.OUTTAKE;
            sleep(400);
            bot.turret.runToAutoOuttakeRight(bot.getIMU());
            bot.slides.runToTopTeleOp();
        });
        armOuttake.start();

    }


    private void drive() {
        if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            bot.resetIMU();
        }
        driveSpeed = 1;
//        driveSpeed -= bot.slides.isHigh()/3;
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        bot.fixMotors();
        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX() , 0);
        if (bot.fieldCentricRunMode) {
            bot.driveFieldCentric(
                    driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed/1.7,
                    (bot.getIMU() + 360) % 360
            );
        } else {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed/1.7
            );
        }
    }

}

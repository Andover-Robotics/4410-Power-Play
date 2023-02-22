package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Claw;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;

    private boolean debugMode = false;
    private boolean cancelPrevAction = false;

    private GamepadEx gp1, gp2;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        bot.initializeImus();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                debugMode = !debugMode;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.START)){
                bot.fieldCentricRunMode = !bot.fieldCentricRunMode;
            }

            telemetry.addData("imu", bot.getIMU());

            if(!debugMode) {//finite state
//                if (bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT) {
//                    if(gp2.getButton(GamepadKeys.Button.A)){
//                        if(!cancelPrevAction) {
//                            bot.claw.close();
//                        }
//                        if(gp2.wasJustPressed(GamepadKeys.Button.X)){
//                            bot.claw.open();
//                            cancelPrevAction = true;
//                        }
//                    }
//                    if(gp2.wasJustReleased(GamepadKeys.Button.A)){
//                        if(!cancelPrevAction){
//                            bot.storage();
//                        }
//                        cancelPrevAction = false;
//                    }
//                } else if (bot.state == Bot.BotState.STORAGE) {
//                    if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
//                        bot.intakeIn();
//                    } else if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
//                        bot.intakeOut();
//                    } else if (gp2.wasJustPressed(GamepadKeys.Button.Y)) {
//                        bot.outtake();
//                    }
//                } else if (bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE) {
//                    if(gp2.getButton(GamepadKeys.Button.A)){
//                        if(!cancelPrevAction) {
//                            bot.secure();
//                        }
//                        if(gp2.wasJustPressed(GamepadKeys.Button.X)){
//                            bot.outtake();
//                            cancelPrevAction = true;
//                        }
//                        if(gp2.wasJustReleased(GamepadKeys.Button.A)){
//                            if(!cancelPrevAction){
//                                bot.claw.open();
//                                bot.storage();
////                                Thread storage = new Thread(() -> {sleep(1000); bot.storage();});
////                                storage.start();
//                            }
//                            cancelPrevAction = false;
//                        }
//                    }
//
//
//
//                    if (gp2.getButton(GamepadKeys.Button.A)) {
//                        bot.secure();
//                    } else {
//                        bot.outtake();
//                    }
//                    if (bot.state == Bot.BotState.SECURE && gp2.wasJustPressed(GamepadKeys.Button.B)) {
//                        bot.claw.open();
//                        bot.storage();
//                    }
//                }
//                double gyro = getIMU() - fieldCentricOffset;
//                bot.turret.runToAngle(0, gyro);//TODO calc angle from thing

            }else{
                bot.horizSlides.runManual(-gp2.getRightY());
                bot.slides.runManual(-gp2.getLeftY());
                bot.turret.runManual(gp2.getLeftX());
                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    bot.resetEncoder();
                }
            }


            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slides.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slides.runToMiddle();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slides.runToLow();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slides.runToBottom();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slides.goDown();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slides.goUp();
            }


            bot.slides.periodic();
            bot.horizSlides.periodic();
            bot.turret.periodic();

            telemetry.addData("drive current", bot.getCurrent());
            telemetry.addData("slide current", bot.slides.getCurrent());
            telemetry.addData("imu", bot.imu0.getAngularOrientation().firstAngle);
            telemetry.addData("getIMU", bot.getIMU());
            telemetry.update();

            drive();
        }
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
                    turnVector.getX() * driveSpeed,
                    bot.getIMU()
            );
        } else {
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed
            );
        }
    }

}



package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;

@Config
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1, cycleTime = 1;

    private boolean debugMode = false;
    private boolean cancelPrevAction = false, autoAlignForward = true, autoMode = false, isRight = false;
    private int index = 4;
    public static double kp = 0.025, ki = 0, kd = 0;

    private PIDController headingAligner = new PIDController(kp, ki, kd);

    Thread thread, otherThread;

    private GamepadEx gp1, gp2;


    @Override
    public void runOpMode() throws InterruptedException {

        headingAligner.setTolerance(1);
        headingAligner.setSetPoint(0);

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
        bot.resetIMU();
        bot.turret.runToIntake(bot.getIMU());

        while (opModeIsActive() && !isStopRequested()) {
            headingAligner.setPID(kp, ki, kd);
            telemetry.addData("cycle", time - cycleTime);
            cycleTime = time;

            gp1.readButtons();
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.BACK)){
                debugMode = !debugMode;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.BACK)){
                autoAlignForward = !autoAlignForward;
            }

            if(!debugMode) {//finite state
                if (bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B) || gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        bot.claw.close();
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X) || gp2.wasJustPressed(GamepadKeys.Button.Y)){
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
                            bot.horizSlides.saveTeleOpIntake();
                            bot.turret.saveIntakePosition(bot.getIMU());
                            goToOuttakeLeft();
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
                        if(!cancelPrevAction){
                            autoMode = true;
                            bot.horizSlides.saveTeleOpIntake();
                            bot.turret.saveIntakePosition(bot.getIMU());
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
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        bot.braceOuttake();
                        otherThread = new Thread(() -> {
                            sleep(200);
                            bot.bringSlidesDown();
                        });
                        otherThread.start();
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                        bot.turret.runToIntake(bot.getIMU());
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                        bot.turret.runToTeleOpOuttakeLeft(bot.getIMU());
                    }else if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        bot.turret.runToTeleOpOuttakeRight(bot.getIMU());
                    }
                } else if (bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B)){
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
                            thread = new Thread(() -> {
                                sleep(400);
                                bot.turret.runToIntake(bot.getIMU());
                            });
                            thread.start();
                            if(autoMode){
                                bot.turret.saveOuttakePosition(bot.getIMU());
                                autoMode = false;
                            }
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        bot.storage();
                    }
                }else if(bot.state == Bot.BotState.BRACE_OUTTAKE || bot.state == Bot.BotState.BRACE_SECURE){
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B)){
                        bot.bringSlidesUp();
                        thread = new Thread(() -> {
                            sleep(200);
                            bot.arm.storage();
                        });
                        thread.start();
                        cancelPrevAction = true;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        cancelPrevAction = false;
                        bot.braceOuttake();
                        otherThread = new Thread(() -> {
                            sleep(200);
                            bot.bringSlidesDown();
                        });
                        otherThread.start();
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.X)){
                        if(!cancelPrevAction) {
                            bot.claw.open();
                            bot.storage();
                            thread = new Thread(() -> {
                                sleep(400);
                                bot.turret.runToIntake(bot.getIMU());
                            });
                            thread.start();
                        }
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                        bot.storage();
                    }
                }
                double rightX = gp2.getRightX(), leftY = gp2.getLeftY();
                bot.turret.runManual(rightX * Math.abs(rightX) / (1 + bot.horizSlides.getPosition()/580.0));
                bot.horizSlides.runManual(leftY * Math.abs(leftY));
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
                double rightX = gp2.getRightX(), leftY = gp2.getLeftY(), rightY = -gp2.getRightX();
                bot.turret.runManual(rightX * Math.abs(rightX) / (1 + bot.horizSlides.getPosition()/580.0));
                bot.horizSlides.runManual(leftY * Math.abs(leftY));
                bot.slides.runManual(rightY * Math.abs(rightY));

                if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
                    bot.resetEncoder();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                    Bot.instance = null;
                    bot = Bot.getInstance(this);
                    bot.slides.resetProfiler();
                    bot.horizSlides.resetProfiler();
                }


                if (bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B) || gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        bot.claw.close();
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X) || gp2.wasJustPressed(GamepadKeys.Button.Y)){
                        bot.claw.open();
                        cancelPrevAction = true;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.A) || gp2.wasJustReleased(GamepadKeys.Button.B)){
                        if(!cancelPrevAction){
                            bot.storage();
                            debugMode = false;
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustReleased(GamepadKeys.Button.LEFT_BUMPER) || gp2.wasJustReleased(GamepadKeys.Button.RIGHT_BUMPER)){
                        if(!cancelPrevAction){
                            bot.horizSlides.saveAutoIntake();
                            bot.turret.saveAutoIntake(isRight, bot.getIMU());
                            goToStackOuttake();
                        }
                        cancelPrevAction = false;
                    }
                } else if (bot.state == Bot.BotState.STORAGE) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                        isRight = true;
                        bot.turret.runToAutoIntakeRight(bot.getIMU());
                        thread = new Thread(() -> {
                            sleep(600);
                            bot.sideStackIntake(index);
                        });
                        thread.start();
                    }else if(gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                        isRight = false;
                        bot.turret.runToAutoIntakeLeft(bot.getIMU());
                        thread = new Thread(() -> {
                            sleep(600);
                            bot.sideStackIntake(index);
                        });
                        thread.start();
                    }
                } else if (bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE) {
                    if(gp2.wasJustPressed(GamepadKeys.Button.A) || gp2.wasJustPressed(GamepadKeys.Button.B)){
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
                            bot.turret.saveAutoOuttake(isRight, bot.getIMU());
                            index--;
                            goToStackIntake();
                        }
                        cancelPrevAction = false;
                    }
                    if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                        bot.storage();
                        debugMode = false;
                    }
                }

                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    index++;
                    if (index > 4) {
                        index = 4;
                    }
                }
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
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
            telemetry.addData("autoalign", autoAlignForward);
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
        thread = new Thread(() ->{
            bot.storage();
            bot.state = Bot.BotState.OUTTAKE;
            sleep(600);
            bot.turret.runToTeleOpOuttakeLeft(bot.getIMU());
            bot.slides.runToTopTeleOp();
            bot.horizSlides.runToFullIn();
            sleep(timeSlidesUp);
            bot.arm.outtake();
        });
        thread.start();

    }

    private void goToOuttakeLeft() {//TODO change values to use the stored values
        thread = new Thread(() ->{
            bot.storage();
            bot.state = Bot.BotState.OUTTAKE;
            sleep(600);
            bot.turret.runToTeleOpOuttakeRight(bot.getIMU());
            bot.slides.runToTopTeleOp();
            bot.horizSlides.runToFullIn();
            sleep(timeSlidesUp);
            bot.arm.outtake();
        });
        thread.start();
    }

    private void goToStackOuttake() {
        thread = new Thread(() -> {
            bot.slides.runToLow();
            bot.arm.autoStorage();
            if(index > 0) {
                sleep(timeIntakeUp);
            }
            bot.horizSlides.runToFullIn();
            sleep(timeIntakeIn);
            if(isRight){
                bot.turret.runToAutoOuttakeRight(bot.getIMU());
            }else{
                bot.turret.runToAutoOuttakeLeft(bot.getIMU());
            }
            bot.slides.runToTop();
            sleep(timeSlidesUp);
            bot.outtake();
        });
        thread.start();
    }

    private void goToStackIntake(){
        thread = new Thread(() -> {
            sleep(400);
            if(isRight) {
                bot.turret.runToAutoIntakeRight(bot.getIMU());
            }else{
                bot.turret.runToAutoIntakeLeft(bot.getIMU());
            }
            sleep(timeSlidesDown);
            bot.claw.open();
            bot.arm.intakeAuto(index);
            sleep(timeIntakeDown);
            bot.horizSlides.runToAutoIntake();
            bot.state = Bot.BotState.INTAKE;
        });
        thread.start();
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
        if(autoAlignForward) {
            double power = headingAligner.calculate(bot.getIMU());
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    -power
            );
        }else{
            bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                    driveVector.getY() * driveSpeed,
                    turnVector.getX() * driveSpeed / 1.7
            );
        }
    }

}

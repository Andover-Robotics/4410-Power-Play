package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;
import org.firstinspires.ftc.teamcode.teleop.subsystems.Claw;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        bot = Bot.getInstance(this);
        GamepadEx gp2 = new GamepadEx(gamepad2);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gp1.readButtons();
            gp2.readButtons();

            if(bot.state == Bot.BotState.INTAKE || bot.state == Bot.BotState.INTAKE_OUT){
                if(bot.claw.getDistance() < Claw.proximityBound){
                    if(gp2.getButton(GamepadKeys.Button.A)){
                        bot.claw.close();
                        Thread goToOuttake = new Thread(() -> {sleep(500); bot.outtake();});
                        goToOuttake.start();
                    }else if(gp2.getButton(GamepadKeys.Button.X)){
                        bot.claw.close();
                        Thread goToStorage = new Thread(() -> {sleep(500); bot.storage();});
                        goToStorage.start();
                    }
                }
            }else if(bot.state == Bot.BotState.STORAGE){
                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    bot.intakeIn();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.X)) {
                    bot.intakeOut();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    bot.frontOuttake();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.B)){
                    bot.outtake();
                }
            }
            else if(bot.state == Bot.BotState.OUTTAKE || bot.state == Bot.BotState.SECURE){
                if(gp2.getButton(GamepadKeys.Button.A)){
                    bot.secure();
                }else{
                    bot.outtake();
                }
                if(bot.state == Bot.BotState.SECURE && gp2.wasJustPressed(GamepadKeys.Button.B)){
                    bot.claw.open();
                    bot.storage();
                }
            }else if(bot.state == Bot.BotState.FRONTOUTTAKE || bot.state == Bot.BotState.FRONTSECURE){
                if(gp2.getButton(GamepadKeys.Button.A)){
                    bot.frontSecure();
                }else{
                    bot.frontOuttake();
                }
                if(bot.state == Bot.BotState.FRONTSECURE && gp2.wasJustPressed(GamepadKeys.Button.B)){
                    bot.claw.open();
                    bot.storage();
                }
            }


            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                bot.slide.runToTop();
            } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                bot.slide.runToMiddle();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                bot.slide.runToLow();
            }else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                bot.slide.runToBottom();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                bot.slide.goDown();
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                bot.slide.goUp();
            }


            bot.slide.periodic();


            // bot.turret.rotate(gp2.getLeftX());

            //TODO test sensor
            telemetry.addData("sensor", bot.claw.getDistance());
            telemetry.addData("drive current", bot.getCurrent());
            telemetry.addData("slide current", bot.slide.getCurrent());
            telemetry.update();

            driveSpeed = 0.6;
            driveSpeed -= bot.slide.isHigh()/3;
            driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            driveSpeed = Math.max(0, driveSpeed);
            bot.fixMotors();
            bot.drive(gp1.getLeftX() * driveSpeed, -gp1.getLeftY() * driveSpeed, gp1.getRightX() * driveSpeed/2);
        }
    }
}



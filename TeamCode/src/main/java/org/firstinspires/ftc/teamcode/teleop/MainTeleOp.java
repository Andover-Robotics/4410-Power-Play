package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.subsystems.Bot;


@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    private Bot bot;
    private double driveSpeed = 1;
    private boolean isManual = false;

    @Override
    public void runOpMode() throws InterruptedException {
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
//            if (gp2.getButton(GamepadKeys.Button.A)) {
//                bot.intake.run();
//            } else if (gp2.getButton(GamepadKeys.Button.B)) {
//                bot.intake.spit();
//            } else {
//                bot.intake.stop();
//            }

            if (gp2.wasJustPressed(GamepadKeys.Button.Y))
                bot.claw.open();
            else if (gp2.wasJustPressed(GamepadKeys.Button.B))
                bot.claw.openRight();
            else if (gp2.wasJustPressed(GamepadKeys.Button.X))
                bot.claw.openLeft();
            else if(gp2.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                bot.claw.close();
            else if (bot.claw.getDistance() < 100)
                if(gp2.getButton(GamepadKeys.Button.Y))
                    bot.claw.open();
                else if(gp2.getButton(GamepadKeys.Button.A))
                    bot.claw.close();

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
                bot.slide.runToTop();
//                bot.claw.open();
//                bot.slide.runToBottom();
            else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
                bot.slide.runToMiddle();
            else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
                bot.slide.runToLow();
            else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
                bot.slide.runToBottom();

//            }else if(gp2.wasJustPressed(GamepadKeys.Button.A)) {
//                bot.slide.runToCone();
//            }

            if (gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER))
                bot.slide.goDown();

            if (gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
                bot.slide.goUp();

            if(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
                isManual = true;
                bot.slide.runPower(gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
            }else{
                if (isManual) {
                    isManual = false;
                    bot.slide.stopManual();
                }
                bot.slide.periodic();
            }


            // bot.turret.rotate(gp2.getLeftX());

            //TODO test sensor
            telemetry.addData("semsor", bot.claw.getDistance());
            telemetry.update();

            FtcDashboard dash = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("strafeSpeed", gp1.getRightX());
            packet.put("forwardSpeed", gp1.getLeftY());
            packet.put("turnSpeed", -gp1.getLeftX());
            dash.sendTelemetryPacket(packet);

            driveSpeed = 0.7;
            driveSpeed -= bot.slide.isHigh()/3;
            driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            driveSpeed = Math.max(0, driveSpeed);
            bot.fixMotors();
            bot.drive(gp1.getLeftX() * driveSpeed, -gp1.getLeftY() * driveSpeed, gp1.getRightX() * driveSpeed);
            }
        }
    }



package org.firstinspires.ftc.teamcode.auto.opmodes;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.vision.OpenCVPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "MainAuto for now", group = "Competition")
public class TestCV extends LinearOpMode {
    OpenCvCamera camera;

//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    TelemetryPacket packet= new TelemetryPacket();

    MotorEx frontLeft= new MotorEx(hardwareMap, "motorFL");
    MotorEx backLeft= new MotorEx(hardwareMap, "motorBL");
    MotorEx backRight= new MotorEx(hardwareMap, "motorBR");
    MotorEx frontRight= new MotorEx(hardwareMap, "motorFR");


    public void strafeRight(double power, int sleepTime) throws InterruptedException {
        frontLeft.set(power);
        backLeft.set(-power);
        frontRight.set(power);
        backRight.set(-power);
        sleep(sleepTime);
    }
    public void straight(double power, int sleepTime) throws InterruptedException {
        frontLeft.set(power);
        backLeft.set(power);
        frontRight.set(power);
        backRight.set(power);
        sleep(sleepTime);
    }


    public void runOpMode() throws InterruptedException{
        Telemetry telemetry1 = null;
        int cameraMonitorViewId= hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id",hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCVPipeline pipeline = new OpenCVPipeline(telemetry1);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,640,OpenCvCameraRotation.UPSIDE_DOWN);
                telemetry1.addLine("Camera Status: Opened");
                telemetry1.update();
                // center = (160, 320)
                // (120, 360) (200, 360)
            }

            @Override
            public void onError(int errorCode) {
                telemetry1.addData("Error code", errorCode);
                telemetry1.update();
            }
        });


        backRight.setInverted(true);
        frontRight.setInverted(true);
        backLeft.setInverted(false);
        frontLeft.setInverted(false);
        backRight.setRunMode(MotorEx.RunMode.RawPower);
        frontLeft.setRunMode(MotorEx.RunMode.RawPower);
        frontRight.setRunMode(MotorEx.RunMode.RawPower);
        backLeft.setRunMode(MotorEx.RunMode.RawPower);

        waitForStart();

        if(pipeline.getSignalVal() == OpenCVPipeline.SignalVal.GREEN){
            strafeRight(0.2, 1);
            straight(0.2,1);
            telemetry1.addData("The signal value is green with a percentage of", OpenCVPipeline.greenPercentage);

        }else if(pipeline.getSignalVal() ==OpenCVPipeline.SignalVal.YELLOW){
            straight(0.2,1);
            telemetry1.addData("The signal value is yellow with a percentage of", OpenCVPipeline.yellowPercentage);

        }else if(pipeline.getSignalVal() ==OpenCVPipeline.SignalVal.PINK){
            strafeRight(-0.2,1);
            straight(0.2,1);
            telemetry1.addData("The signal value is pink with a percentage of", OpenCVPipeline.pinkPercentage);

        }

        camera.stopStreaming();
        //REHEHEHEHEHEHEH


    }

}
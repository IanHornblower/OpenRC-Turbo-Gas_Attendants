package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Config
public class FreightFrenzyCamera {

    // TODO: Add Vertical Lines for Object Divisions

    public enum position{A, B, C, ABSENT}

    private OpenCvCamera cam;
    ContourPipeline pipeline;
    HardwareMap hwMap;

    private static int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static int lowestBlobArea = 200;
    public static int abVerticalLine = 213; //213
    public static int bcVerticalLine = 426; // 426

    public static int topBorderOffset = 5;
    public static int lineLength = 335;

    public static int borderLeftX   = 5;   //amount of pixels from the left side of the cam to skip
    public static int borderRightX  = 5;   //amount of pixels from the right of the cam to skip
    public static int borderTopY    = 5;   //amount of pixels from the top of the cam to skip
    public static int borderBottomY = 25;   //amount of pixels from the bottom of the cam to skip

    // Color Range (Pink)                              Y      Cr     Cb
    private static Scalar scalarLowerYCrCb = new Scalar(195.0, 0, 0);
    private static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);

    public FreightFrenzyCamera(HardwareMap hwMap) {
        this.hwMap = hwMap;
    }

    public FreightFrenzyCamera (HardwareMap hwMap,
                                int CAMERA_WIDTH, int CAMERA_HEIGHT,
                                Scalar lower, Scalar upper) {
        this.hwMap = hwMap;
        this.CAMERA_WIDTH  = CAMERA_WIDTH;
        this.CAMERA_HEIGHT = CAMERA_HEIGHT;

        this.scalarLowerYCrCb = lower;
        this.scalarUpperYCrCb = upper;
    }

    public void initPhoneCamera() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
    }

    public void initWebCamera() {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    }

    public void initPipeline() {
        cam.setPipeline(pipeline = new ContourPipeline());

        pipeline.ConfigurePipeline(borderLeftX, borderRightX, borderTopY, borderBottomY,  CAMERA_WIDTH, CAMERA_HEIGHT);
        pipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
    }

    public void refreshScalars() {
        pipeline.ConfigureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.ConfigureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
    }

    public void startCamera() {
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }

    public OpenCvCamera getCamera() {
        return cam;
    }

    public void startCameraStream(int fps) {
        FtcDashboard.getInstance().startCameraStream(cam, fps);
    }

    public position determinePosition() {
        if(pipeline.getRectArea() > lowestBlobArea){
            if(pipeline.getRectMidpointX() > bcVerticalLine){
                return position.C;
            }
            else if(pipeline.getRectMidpointX() > abVerticalLine){
                return position.B;
            }
            return position.A;
        }
        return position.ABSENT;
    }

    public String sDeterminePosition() {
        if(pipeline.getRectArea() > lowestBlobArea){
            if(pipeline.getRectMidpointX() > bcVerticalLine){
                return "C";
            }
            else if(pipeline.getRectMidpointX() > abVerticalLine){
                return "B";
            }
            return "A";
        }
        return "None";
    }

    public void stopCameraStream() {
         cam.stopStreaming();
    }

    public void closeCamera() {
        cam.closeCameraDevice();
    }

    public void shutdownPipeline() {
        stopCameraStream();
        closeCamera();
    }

    /**
    public void sendTelemetry() {
        telemetry.addData("Frame Count", phoneCam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
        telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());

        telemetry.addData("RectArea: ", pipeline.getRectArea());
        telemetry.update();
    }
     */

    public static int getCameraWidth() {
        return CAMERA_WIDTH;
    }

    public static int getCameraHeight() {
        return CAMERA_HEIGHT;
    }

    public static Scalar getScalarLowerYCrCb() {
        return scalarLowerYCrCb;
    }

    public static Scalar getScalarUpperYCrCb() {
        return scalarUpperYCrCb;
    }
}

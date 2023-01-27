package org.firstinspires.ftc.teamcode.Logic.AutonomousLogic;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class OpenCV extends Thread {
    public static OpenCvWebcam webcam = null;
    public static Pipeline pipeline = null;
    public static Telemetry telemetry;

    public static double time_for_recognition = 6;

    public OpenCV(HardwareMap map, Telemetry telem, String webcam_name) { // Initialize variable before WaitForStart()
            // After the code is initialized but before it's run, we can view the camera stream in the driver hub
            // It only updates when you press it, however - and it obviously only adds one frame per such refresh
        telemetry = telem;

        webcam = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, webcam_name), map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName()));

        pipeline = new Pipeline();

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(3000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT); // 720p maximum
                                // 1280 x 960 is maximum
                                // currently only 120p I think
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera failed to open");
            }
        });
        webcam.resumeViewport();
        telemetry.addLine("OpenCV Initialized");
        telemetry.update();
    }

    public static String readQrCode() {

        String result = pipeline.getQR();

        double startTime = System.currentTimeMillis();

        while (result.equals("") && System.currentTimeMillis() - startTime < time_for_recognition * 1000) {
            telemetry.addData("", System.currentTimeMillis() - startTime);
            telemetry.update();
            result = pipeline.getQR();
        }

        return result;
    }


    public static String getColor() {

        String result = pipeline.getColor();

        double startTime = System.currentTimeMillis();

        while (result.equals("") && System.currentTimeMillis() - startTime < time_for_recognition * 1000) {
            telemetry.addData("", System.currentTimeMillis() - startTime);
            telemetry.update();
            result = pipeline.getColor();
        }

        return result;
    }

    private static boolean should_be_running = true;

    public static void stopStreaming() {
        webcam.stopStreaming();
        should_be_running = false;
    }

    public void run() {
        while (should_be_running) {
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs()); // this might be extremely long
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("Yellow", pipeline.pct_yellow);
            telemetry.addData("Cyan", pipeline.pct_cyan);
            telemetry.addData("Purple", pipeline.pct_purple);
            telemetry.update();
        }
    }

}

class Pipeline extends OpenCvPipeline {

    private static String qr_result, color_result;
    private static QRCodeDetector detector = new QRCodeDetector();

    public static double pct_yellow, pct_cyan, pct_purple;

    private static final int
            yellow_hue = 30,
            cyan_hue = 90,
            purple_hue = 135;

    private static final int[] hues = {
            yellow_hue, cyan_hue, purple_hue
    };

    private final static double hue_tolerance = 20, min_area = 0.1;
            // min area: what proportion of the screen must match target color

    public Pipeline() {
        super();
    }

    @Override
    public Mat processFrame(Mat input) {

        // process QR codes
        qr_result = "";
        // qr_result = detector.detectAndDecodeCurved(input);

        // color detection
        Mat hsv_img = new Mat();
        Imgproc.cvtColor(input, hsv_img, Imgproc.COLOR_RGB2HSV); // Convert to HSV image
                                                    // For some reason OpenCV uses BGR?????
        color_result = "";                          // may have to be RGB not BGR if there's an issue
        // if something is wrong, we assume there's nothing
        if (hsv_img.empty()) {
            return input;
        }

        double[] areas = area_of_hues(hsv_img, hues);
        pct_yellow = areas[0];
        pct_cyan = areas[1];
        pct_purple = areas[2];

        if (pct_yellow > min_area) {
            color_result = "yellow";
        } else if (pct_cyan > min_area) {
            color_result = "cyan";
        } else if (pct_purple > min_area) {
            color_result = "purple";
        }

        return input;

    }

    public static String getQR() {
        return qr_result;
    }

    public static String getColor() {
        return color_result;
    }

    public static double[] area_of_hues(Mat hsv_img, int[] hues) {
        double[] areas = new double[hues.length];
        for (int i = 0; i < hsv_img.rows(); i++)
            for (int j = 0; j < hsv_img.cols(); j++)
                if ((hsv_img.get(i, j)[1] > 100) && (hsv_img.get(i, j)[2] > 100)) // if hue & saturation both > 100
                    for (int k = 0; k < hues.length; k++)
                        if (Math.abs((hsv_img.get(i, j)[0] - hues[k] + 128) % 256 - 128) < hue_tolerance)
                            areas[k] += 1;
                        // we have to do % 256 because otherwise 250 and 0 wouldn't register as 6 apart
        for (int i = 0; i < hues.length; i++) {
            areas[i] /= (hsv_img.rows() * hsv_img.cols());
            areas[i] = ((int) (areas[i] * 10 + 0.5)) * 0.1;
        }
        return areas;
    }
}
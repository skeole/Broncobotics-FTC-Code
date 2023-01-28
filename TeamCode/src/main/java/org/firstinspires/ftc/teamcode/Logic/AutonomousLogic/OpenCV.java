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

abstract class CameraConstants {
    public static final int
        width = 160, // width and height must form a pair that the camera can get images for
        height = 120,
            /* All resolution pairs that work:
                * 160 by 120 (4 : 3)
                * 176 by 144 (11 : 9)
                * 320 by 176 (20 : 11)
                * 320 by 240 (4 : 3)
                * 352 by 288 (11 : 9)
                * 432 by 240 (9 : 5)
                * 544 by 288 (17 : 9)
                * 640 by 360 (16 : 9)
                * 640 by 480 (4 : 3)
                * 752 by 416
                * 800 by 448
                * 800 by 600 (4 : 3)
                * 864 by 480 (9 : 5)
                * 960 by 544
                * 960 by 720 (4 : 3)
                * 1024 by 576 (16 : 9)
                * 1184 by 656
                * 1280 by 720 (16 : 9)
                * 1280 by 960 (4 : 3)
             */
        yellow_hue = 24,
        cyan_hue = 99,
        purple_hue = 136,
        min_saturation = 60,
        min_value = 80,
        hue_tolerance = 12;

    public static final double min_area = 0.1;

    public static final boolean
        show_webcam_data = false,
        qr_sensing = false,
        color_sensing = true;
}
public class OpenCV extends Thread {
    public static OpenCvWebcam webcam = null;
    public static Pipeline pipeline = null;
    public static Telemetry telemetry = null;

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
                webcam.startStreaming(CameraConstants.width, CameraConstants.height, OpenCvCameraRotation.UPRIGHT); // 720p maximum
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
    private static boolean should_be_running = true;

    public static void stopStreaming() {
        webcam.stopStreaming();
        should_be_running = false;
    }

    public void run() {
        while (should_be_running) {
            if (CameraConstants.show_webcam_data) {
                telemetry.addData("Frame Count", webcam.getFrameCount());
                telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
                telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
                telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs()); // this might be extremely long
                telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
                telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
                telemetry.addLine();
            }
            if (CameraConstants.qr_sensing) {
                telemetry.addData("QR", pipeline.getQR());
                telemetry.addLine();
            }
            if (CameraConstants.color_sensing) {
                telemetry.addData("Yellow", pipeline.pct_yellow);
                telemetry.addData("Cyan", pipeline.pct_cyan);
                telemetry.addData("Purple", pipeline.pct_purple);
                telemetry.addLine();
                telemetry.addData("Color", pipeline.getColor());
                telemetry.addLine();
            }
            telemetry.update();
        }
    }

}

class Pipeline extends OpenCvPipeline {

    private static String qr_result = "None", color_result = "None";
    private static QRCodeDetector detector = new QRCodeDetector();
    public static double pct_yellow, pct_cyan, pct_purple;

    private static final int[] hues = {
            CameraConstants.yellow_hue, CameraConstants.cyan_hue, CameraConstants.purple_hue
    };

    public Pipeline() {
        super();
    }

    @Override
    public Mat processFrame(Mat input) {

        if (CameraConstants.qr_sensing) {
            // process QR codes
            qr_result = "None";
            qr_result = detector.detectAndDecodeCurved(input); // don't run both because it does make it run slower
        }

        if (CameraConstants.color_sensing) {
            // color detection
            Mat hsv_img = new Mat();
            Imgproc.cvtColor(input, hsv_img, Imgproc.COLOR_RGB2HSV); // Convert to HSV image
                                                        // For some reason OpenCV uses BGR?????
            color_result = "None";                      // may have to be RGB not BGR if there's an issue
            // if something is wrong, we assume there's nothing
            if (hsv_img.empty()) {
                return input;
            }

            double[] areas = area_of_hues(hsv_img, hues);
            pct_yellow = areas[0];
            pct_cyan = areas[1];
            pct_purple = areas[2];

            if (pct_yellow > CameraConstants.min_area) {
                color_result = "Yellow";
            } else if (pct_cyan > CameraConstants.min_area) {
                color_result = "Cyan";
            } else if (pct_purple > CameraConstants.min_area) {
                color_result = "Purple";
            }
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
                if ((hsv_img.get(i, j)[1] > CameraConstants.min_saturation) && (hsv_img.get(i, j)[2] > CameraConstants.min_value)) // if hue & saturation both exceed the minima
                    for (int k = 0; k < hues.length; k++)
                        if (Math.abs((hsv_img.get(i, j)[0] - hues[k] + 128) % 256 - 128) < CameraConstants.hue_tolerance)
                            areas[k] += 1;
                        // we have to do % 256 because otherwise 250 and 0 wouldn't register as 6 apart
        for (int i = 0; i < hues.length; i++) {
            areas[i] /= (hsv_img.rows() * hsv_img.cols());
            areas[i] = ((int) (areas[i] * 100 + 0.5)) * 0.01;
        }
        return areas;
    }
}
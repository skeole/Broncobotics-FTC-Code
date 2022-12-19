package org.firstinspires.ftc.teamcode.Logic.AutonomousLogic;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Logic.RobotHardware;
import org.firstinspires.ftc.teamcode.Robots;

import java.util.ArrayList;
import java.util.List;

public class Tensorflow {

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private HardwareMap hardwareMap;

    private final float min_confidence;
    private final int input_size;
    private final boolean useAsset;
    private final String VUFORIA_KEY;
    private final String webcam_name;

    private final String TFOD_MODEL_ASSET;
    private final String TFOD_MODEL_FILE;

    private final String[] LABELS;

    public Tensorflow(RobotHardware rh) {
        hardwareMap = rh.map;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(rh.camera_zoom, 16.0 / 9.0);
            //camera_zoom must be >= 1.0
        }
        min_confidence = rh.min_confidence;
        input_size = rh.input_size;
        useAsset = rh.useAsset;
        VUFORIA_KEY = rh.VUFORIA_KEY;
        webcam_name = rh.webcam_name;

        TFOD_MODEL_ASSET = rh.TFOD_MODEL_ASSET;
        TFOD_MODEL_FILE = rh.TFOD_MODEL_FILE;

        LABELS = rh.LABELS;
    }

    public boolean canSee(String name) {
        return getData().contains(name);
    }

    public ArrayList<String> getUpdatedData() {
        return getData(true, -1000000.0, 1000000.0, -1000000.0, 1000000.0);
    }

    public ArrayList<String> getData() {
        return getData(false, -1000000.0, 1000000.0, -1000000.0, 1000000.0);
    }

    public ArrayList<String> getUpdatedData(double min_x, double max_x, double min_y, double max_y) {
        return getData(true, min_x, max_x, min_y, max_y);
    }

    public ArrayList<String> getData(double min_x, double max_x, double min_y, double max_y) {
        return getData(false, min_x, max_x, min_y, max_y);
    }

    public ArrayList<String> getData(boolean updated, double min_x, double max_x, double min_y, double max_y) {
        List<Recognition> recognitions;
        if (updated) {
            recognitions = getRawUpdatedData();
        } else {
            recognitions = getRawData();
        }
        ArrayList<String> data = new ArrayList<String>();
        if (recognitions != null) {
            for (Recognition recognition : recognitions) {
                //only add data if all points are within the boundary rectangle we chose
                if (((min_x < recognition.getLeft()) && (recognition.getRight() < max_x)) && ((min_y < recognition.getBottom()) && (recognition.getTop() < max_y))) {
                    data.add(recognition.getLabel());
                }
            }
        }
        return data;
    }

    public List<Recognition> getRawData(){
        if (tfod != null) {
            //returns all data
            return tfod.getRecognitions();
        }
        return null;
    }

    public List<Recognition> getRawUpdatedData(){
        if (tfod != null) {
            //only returns the NEW information
            return tfod.getUpdatedRecognitions();
        }
        return null;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, webcam_name);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = min_confidence;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = input_size;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        if (useAsset) {
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        } else {
            tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        }
    }

}
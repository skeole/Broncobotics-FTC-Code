package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import java.util.ArrayList;
import java.util.Arrays;

public class Robots {
    public void init201() {
        dc_motor_names = new ArrayList<>(Arrays.asList("Left", "Right"));
        max_power = new double[] {0.8, 0.8};
        min_power = new double[] {-0.8, -0.8};
        motor_max_positions = new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        motor_min_positions = new double[] {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        invert_dc_motors = new boolean[] {true, false};
        p_weights = new double[] {0.05, 0.05};
        d_weights = new double[] {0.1, 0.1};

        servo_names = new ArrayList<>(Arrays.asList());
        servo_max_positions = new double[] {};
        servo_min_positions = new double[] {};

        positions = new double[][] {
                {0, 100, 200, 300}, //Arm Heights
                {0.2, 0.6} //Claw
        };

        cr_servo_names = new ArrayList<>(Arrays.asList("Virtual"));
        invert_cr_servos = new boolean[] {false};

        distance_sensor_names = new ArrayList<>(Arrays.asList());

        touch_sensor_names = new ArrayList<>(Arrays.asList());

        color_sensor_names = new ArrayList<>(Arrays.asList());

        led_names = new ArrayList<>(Arrays.asList());

        strafe = -0.8;
        turning_weight = -1.0;
        distance_weight = -1.0;

        locked_motion = false;
        locked_rotation = false;

        usePID = true;
        p_weight = 0.025;
        d_weight = 0.85;

        axesOrder = AxesOrder.ZYX;
        invertIMU = false;

        useRoadRunner = false;
        distance_weight_two = 1.0;
        ticks_per_revolution = 0.0;
        wheel_radius = 0.0;
        gear_ratio = 0.0;
        lateral_distance = 0.0;
        forward_offset = 4.0;
        integer_overflow = false;

        forward_multiplier = 1.0;
        strafing_multiplier = 1.0;
        turning_multiplier = 1.0;

        robot_width = 10;
        robot_length = 18;

        encoderNames = new ArrayList<>(Arrays.asList("leftEncoder", "rightEncoder", "frontEncoder"));
        invert_encoders = new boolean[] {false, false, false};
        //ordered by left, right, front
        wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
        //ordered by right front, right back, left back, left front
    }

    public void init202() {
        dc_motor_names = new ArrayList<>(Arrays.asList());
        max_power = new double[] {};
        min_power = new double[] {};
        motor_max_positions = new double[] {};
        motor_min_positions = new double[] {};
        invert_dc_motors = new boolean[] {};
        p_weights = new double[] {};
        d_weights = new double[] {};

        servo_names = new ArrayList<>(Arrays.asList());
        servo_max_positions = new double[] {};
        servo_min_positions = new double[] {};

        positions = new double[][] {
                {0, 100, 200, 300}, //Arm Heights
                {0.2, 0.6} //Claw
        };

        cr_servo_names = new ArrayList<>(Arrays.asList());
        invert_cr_servos = new boolean[] {};

        distance_sensor_names = new ArrayList<>(Arrays.asList());

        touch_sensor_names = new ArrayList<>(Arrays.asList());

        color_sensor_names = new ArrayList<>(Arrays.asList());

        led_names = new ArrayList<>(Arrays.asList());

        strafe = 0.0;
        turning_weight = 0.0;
        distance_weight = 1.0;

        locked_motion = false;
        locked_rotation = false;

        usePID = false;
        p_weight = 0.025;
        d_weight = 0.85;

        axesOrder = AxesOrder.XZY;
        invertIMU = false;

        useRoadRunner = false;
        distance_weight_two = 1.0;
        ticks_per_revolution = 0.0;
        wheel_radius = 0.0;
        gear_ratio = 0.0;
        lateral_distance = 0.0;
        forward_offset = 0.0;
        integer_overflow = false;

        forward_multiplier = 1.0;
        strafing_multiplier = 1.0;
        turning_multiplier = 1.0;

        robot_width = 10;
        robot_length = 18;

        encoderNames = new ArrayList<>(Arrays.asList("leftEncoder", "rightEncoder", "frontEncoder"));
        invert_encoders = new boolean[] {false, false, false};
        //ordered by left, right, front
        wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
        //ordered by right front, right back, left back, left front
    }

    public ArrayList<String> dc_motor_names; public double[] max_power; public double[] min_power;
        public double[] motor_max_positions; public double[] motor_min_positions;
        public boolean[] invert_dc_motors; public double[] p_weights; public double[] d_weights;

    public ArrayList<String> servo_names; public double[] servo_max_positions; public double[] servo_min_positions;

    public double[][] positions;

    public ArrayList<String> cr_servo_names; public boolean[] invert_cr_servos;

    public ArrayList<String> distance_sensor_names;

    public ArrayList<String> touch_sensor_names;

    public ArrayList<String> color_sensor_names;

    public ArrayList<String> led_names;

    //Driving
    public double strafe; public double turning_weight; public double distance_weight;
    public boolean locked_motion; public boolean locked_rotation;

    //PID
    public boolean usePID; public double p_weight; public double d_weight;
    public AxesOrder axesOrder; public boolean invertIMU;

    //Road Runner
    public boolean useRoadRunner; public double distance_weight_two; public double ticks_per_revolution;
    public double wheel_radius; public double gear_ratio;
    public double lateral_distance; public double forward_offset;
    public boolean integer_overflow;

    //Road Runner Tuning
    public double forward_multiplier; public double strafing_multiplier; public double turning_multiplier;

    //Collision Detection
    public double robot_width; public double robot_length;

    //Tensorflow
    public String VUFORIA_KEY =
            "Vuforia Key";
    public String webcam_name = "Webcam 1";
    public double camera_zoom = 1.0;
    public String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Sleeve_Detection.tflite"; //Move Sleeve_Detection.tflite to FtcRobotController/assets I think
    public String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    public Float min_confidence = 0.75f;
    public int input_size = 300;
    public boolean useAsset = true;
    public String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public ArrayList<String> encoderNames; //REV Thorough Bore Encoders
    public boolean[] invert_encoders;
    public ArrayList<String> wheel_names;

    public ArrayList<String> keys = new ArrayList<>(Arrays.asList(
            "operator a", "operator b", "operator x", "operator y", "operator dpad_up", "operator dpad_down",
            "operator dpad_left", "operator dpad_right", "operator left_bumper", "operator right_bumper",
            "driver a", "driver b", "driver x", "driver y", "driver dpad_up", "driver dpad_down",
            "driver dpad_left", "driver dpad_right", "driver left_bumper", "driver right_bumper",
            "operator left_stick_x", "operator right_stick_x", "operator left_stick_y", "operator right_stick_y",
            "operator left_trigger", "operator right_trigger", "driver left_trigger"
    )); //DO NOT CHANGE THIS

    public void init_base() { //copy this if you want to set up a different robot
        dc_motor_names = new ArrayList<>(Arrays.asList());
        max_power = new double[] {};
        min_power = new double[] {};
        motor_max_positions = new double[] {}; //Double.POSITIVE_INFINITY
        motor_min_positions = new double[] {}; //Double.NEGATIVE_INFINITY
        invert_dc_motors = new boolean[] {};
        p_weights = new double[] {};
        d_weights = new double[] {};

        servo_names = new ArrayList<>(Arrays.asList());
        servo_max_positions = new double[] {};
        servo_min_positions = new double[] {};

        cr_servo_names = new ArrayList<>(Arrays.asList());
        invert_cr_servos = new boolean[] {};

        distance_sensor_names = new ArrayList<>(Arrays.asList());

        touch_sensor_names = new ArrayList<>(Arrays.asList());

        color_sensor_names = new ArrayList<>(Arrays.asList());

        led_names = new ArrayList<>(Arrays.asList());

        strafe = 0.0;
        turning_weight = 0.0;
        distance_weight = 0.0;

        locked_motion = false;
        locked_rotation = false;

        usePID = false;
        p_weight = 0.025;
        d_weight = 0.85;

        axesOrder = AxesOrder.XYZ;
        invertIMU = false;

        useRoadRunner = false;
        distance_weight_two = 0.0;
        ticks_per_revolution = 0.0;
        wheel_radius = 0.0;
        gear_ratio = 0.0;
        lateral_distance = 0.0;
        forward_offset = 0.0;
        integer_overflow = false;

        forward_multiplier = 0.0;
        strafing_multiplier = 0.0;
        turning_multiplier = 0.0;

        robot_width = 0;
        robot_length = 0;

        encoderNames = new ArrayList<>(Arrays.asList("leftEncoder", "rightEncoder", "frontEncoder"));
        invert_encoders = new boolean[] {false, false, false};
        //ordered by left, right, front
        wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
        //ordered by right front, right back, left back, left front
    }
}

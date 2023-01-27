package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import java.util.ArrayList;
import java.util.Arrays;

public class Robots {
    public static void init201() {
        dc_motor_names = new ArrayList<>(Arrays.asList("Left"));
        max_power = new double[] {1.0};
        min_power = new double[] {-1.0};
        motor_max_positions = new double[] {Double.POSITIVE_INFINITY};
        motor_min_positions = new double[] {Double.NEGATIVE_INFINITY};
        invert_dc_motors = new boolean[] {true};
        p_weights = new double[] {0.05};
        d_weights = new double[] {0.001};

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
        max_speed = 1.0; // we should change this depending on how high up the arms are

        locked_motion = false;
        locked_rotation = false;

        use_IMU = true;
        usePID = true;
        p_weight = 0.025;
        d_weight = 0.0085;

        axesOrder = AxesOrder.XYZ;
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

    public static void init202() {
        dc_motor_names = new ArrayList<>(Arrays.asList("joint1right", "joint2"));
        max_power = new double[] {0.4, 0.4};
        min_power = new double[] {-0.4, -0.2};
        motor_max_positions = new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        motor_min_positions = new double[] {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
        invert_dc_motors = new boolean[] {false, false};
        p_weights = new double[] {0.02, 0.02};
        d_weights = new double[] {0.0001, 0.0001}; // d-control?!?!??!?!?!??!??!??!?!??!?

        servo_names = new ArrayList<>(Arrays.asList("clawAligner", "claw"));
        servo_max_positions = new double[] {0.75, 1};
        servo_min_positions = new double[] {0, 0.5};

        positions = new double[][] {
                {0.5, 1} //Claw
        };

        cr_servo_names = new ArrayList<>(Arrays.asList());
        invert_cr_servos = new boolean[] {};

        distance_sensor_names = new ArrayList<>(Arrays.asList());

        touch_sensor_names = new ArrayList<>(Arrays.asList());

        color_sensor_names = new ArrayList<>(Arrays.asList());

        led_names = new ArrayList<>(Arrays.asList());

        strafe = 1.0;
        turning_weight = 1.0;
        distance_weight = 1.0;
        max_speed = 1.0;

        locked_motion = false;
        locked_rotation = false;

        drive_type = 1;

        use_IMU = false;
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

    public static ArrayList<String> dc_motor_names;
    public static double[] max_power, min_power, motor_max_positions, motor_min_positions, p_weights, d_weights;
    public static boolean[] invert_dc_motors;

    public static ArrayList<String> servo_names;
    public static double[] servo_max_positions, servo_min_positions;

    public static double[][] positions;

    public static ArrayList<String> cr_servo_names;
    public static boolean[] invert_cr_servos;

    public static ArrayList<String> distance_sensor_names, touch_sensor_names, color_sensor_names, led_names;

    //Driving
    public static double dead_zone = 0.2, strafe, turning_weight, distance_weight, max_speed;
    public static boolean locked_motion, locked_rotation;
    public static int drive_type = 0; // 0 means normal, 1 or 2 means custom

    //PID
    public static boolean use_IMU, usePID, invertIMU;
    public static double p_weight, d_weight;
    public static AxesOrder axesOrder;

    //Road Runner
    public static boolean useRoadRunner, integer_overflow;
    public static double distance_weight_two, ticks_per_revolution, wheel_radius, gear_ratio, lateral_distance, forward_offset;
    //Road Runner Tuning
    public static double forward_multiplier, strafing_multiplier, turning_multiplier;

    //Collision Detection
    public static double robot_width, robot_length;

    //Tensorflow
    public static String VUFORIA_KEY = "Vuforia Key",
                         webcam_name = "Webcam 1",
                         TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/Sleeve_Detection.tflite", //Move Sleeve_Detection.tflite to FtcRobotController/assets I think
                         TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";
    public static double camera_zoom = 1.0;
    public static Float min_confidence = 0.75f;
    public static int input_size = 300;
    public static boolean useAsset = true;
    public static String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    public static ArrayList<String> encoderNames, wheel_names;
                //REV Thorough Bore Encoders
    public static boolean[] invert_encoders;

    public static final ArrayList<String> keys = new ArrayList<>(Arrays.asList(
            "operator a", "operator b", "operator x", "operator y", "operator dpad_up", "operator dpad_down",
            "operator dpad_left", "operator dpad_right", "operator left_bumper", "operator right_bumper",
            "driver a", "driver b", "driver x", "driver y", "driver dpad_up", "driver dpad_down", "driver dpad_left",
            "driver dpad_right", "driver left_bumper", "driver right_bumper", "operator left_stick_x",
            "operator right_stick_x", "operator left_stick_y", "operator right_stick_y", "operator left_trigger",
            "operator right_trigger", "driver left_trigger"
    )); // DO NOT CHANGE THIS

    public static void init_freight_frenzy() {
        dc_motor_names = new ArrayList<>(Arrays.asList("duckWheel", "arm"));
        max_power = new double[] {0.5, 0.5};
        min_power = new double[] {-0.5, -0.1};
        motor_max_positions = new double[] {Double.POSITIVE_INFINITY, 1800};
        motor_min_positions = new double[] {Double.NEGATIVE_INFINITY, 0};
        invert_dc_motors = new boolean[] {false, false};
        p_weights = new double[] {0, 0.05};
        d_weights = new double[] {0, 0};

        servo_names = new ArrayList<>(Arrays.asList("claw"));
        servo_max_positions = new double[] {1.0};
        servo_min_positions = new double[] {0.0};

        cr_servo_names = new ArrayList<>(Arrays.asList());
        invert_cr_servos = new boolean[] {};

        distance_sensor_names = new ArrayList<>(Arrays.asList("d_sensor"));

        touch_sensor_names = new ArrayList<>(Arrays.asList());

        color_sensor_names = new ArrayList<>(Arrays.asList());

        led_names = new ArrayList<>(Arrays.asList());

        strafe = 1.0;
        turning_weight = 1.0;
        distance_weight = 1.0;
        max_speed = 1.0;

        locked_motion = false;
        locked_rotation = false;

        usePID = false;
        p_weight = 0.025;
        d_weight = 0.85;

        use_IMU = false;
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
        // ordered by left, right, front
        wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
        // ordered by right front, right back, left back, left front
    }

    public static void init_base() { // copy this if you want to set up a different robot
        dc_motor_names = new ArrayList<>(Arrays.asList());
        max_power = new double[] {};
        min_power = new double[] {};
        motor_max_positions = new double[] {}; // Double.POSITIVE_INFINITY
        motor_min_positions = new double[] {}; // Double.NEGATIVE_INFINITY
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

        dead_zone = 0.2;

        strafe = 1.0;
        turning_weight = 1.0;
        distance_weight = 1.0;
        max_speed = 0.0;

        locked_motion = false;
        locked_rotation = false;

        drive_type = 0;

        usePID = false;
        p_weight = 0.025;
        d_weight = 0.85;

        use_IMU = false;
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
        // ordered by left, right, front
        wheel_names = new ArrayList<>(Arrays.asList("rightFront", "rightBack", "leftBack", "leftFront"));
        // ordered by right front, right back, left back, left front
    }

    public static double magnitude(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public static double vectorToAngle(double x, double y) {
        if (x == 0) x = 0.001;
        if (y == 0) y = 0.001;
        return Math.atan(x / y) + Math.PI * (y > 0 ? 0 : 1) * (x > 0 ? 1 : -1);
    }

    public static double[] weird_driving(double lsx, double lsy, double rsx, double rsy, Gamepad gamepad) { // already factors in deadzone
        // should be turning factor, distance factor, offset, Speed factor
        switch (drive_type) {
            case 1: // Felix drive
                double TRIGGER_SENSITIVITY = 0.65, BUTTON_SENSITIVITY = 0.3, LEFT_SENSITIVITY = 1, RIGHT_SENSITIVITY = 0.3;

                double turnAmount = gamepad.left_trigger * TRIGGER_SENSITIVITY + -gamepad.right_trigger * TRIGGER_SENSITIVITY + ((gamepad.left_bumper) ? BUTTON_SENSITIVITY : 0) + (gamepad.right_bumper ? -BUTTON_SENSITIVITY : 0);

                double[] vec = {
                        lsx * LEFT_SENSITIVITY + rsx * RIGHT_SENSITIVITY,
                        lsy * LEFT_SENSITIVITY + rsy * RIGHT_SENSITIVITY
                };

                return new double[] {turnAmount, magnitude(vec[0], vec[1]), vectorToAngle(vec[0], vec[1]), 1};
            case 2: // Tank drive with triggers
                double turning = gamepad.right_trigger - gamepad.left_trigger;
                double forward = gamepad.right_trigger + gamepad.left_trigger;
                return new double[] {turning, forward, 0, 1};
            case 3: // Normal tank drive, with right trigger to slow down
                turning = rsy - lsy;
                forward = rsx + lsx;
                return new double[] {turning, forward, 0, 1 + 2 * gamepad.right_trigger};
            default:
                return new double[] {gamepad.right_stick_x, Math.sqrt(gamepad.left_stick_x * gamepad.left_stick_x + gamepad.left_stick_y * gamepad.left_stick_y), vectorToAngle(gamepad.left_stick_x, gamepad.left_stick_y), 1};
        }
    }
}

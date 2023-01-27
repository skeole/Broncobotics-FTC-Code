package org.firstinspires.ftc.teamcode.Logic;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robots;

public class RobotHardware extends Robots {

    public static BNO055IMU imu;

    public static HardwareMap map;
    public static Telemetry telemetry;

    public static DcMotor[] wheel_list;
    public static DcMotor[] dc_motor_list;
    public static Servo[] servo_list;
    public static CRServo[] cr_servo_list;

    public static DistanceSensor[] distance_sensor_list;
    public static TouchSensor[] touch_sensor_list;
    public static ColorSensor[] color_sensor_list;
    public static RevBlinkinLedDriver[] led_list;

    public static double imu_zero;

    public static void initialize_hardware(HardwareMap hardwareMap, Telemetry telemetry_) {

        wheel_list = new DcMotor[wheel_names.size()];
        dc_motor_list = new DcMotor[dc_motor_names.size()];
        servo_list = new Servo[servo_names.size()];
        cr_servo_list = new CRServo[cr_servo_names.size()];

        distance_sensor_list = new DistanceSensor[distance_sensor_names.size()];
        touch_sensor_list = new TouchSensor[touch_sensor_names.size()];
        color_sensor_list = new ColorSensor[color_sensor_names.size()];
        led_list = new RevBlinkinLedDriver[led_names.size()];

        telemetry = telemetry_;

        if (use_IMU) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            imu_zero = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS).firstAngle;
        }

        for (int i = 0; i < wheel_list.length; i++) {
            wheel_list[i] = hardwareMap.get(DcMotor.class, wheel_names.get(i));
            wheel_list[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel_list[i].setDirection(i > 1 ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        }

        for (int i = 0; i < distance_sensor_list.length; i++)
            distance_sensor_list[i] = hardwareMap.get(DistanceSensor.class, distance_sensor_names.get(i));

        for (int i = 0; i < touch_sensor_list.length; i++)
            touch_sensor_list[i] = hardwareMap.get(TouchSensor.class, touch_sensor_names.get(i));

        for (int i = 0; i < color_sensor_list.length; i++)
            color_sensor_list[i] = hardwareMap.get(ColorSensor.class, color_sensor_names.get(i));

        for (int i = 0; i < led_list.length; i++)
            led_list[i] = hardwareMap.get(RevBlinkinLedDriver.class, led_names.get(i));

        for (int i = 0; i < dc_motor_list.length; i++) {
            dc_motor_list[i] = hardwareMap.get(DcMotor.class, dc_motor_names.get(i));
            dc_motor_list[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            dc_motor_list[i].setDirection(invert_dc_motors[i] ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }

        for (int i = 0; i < servo_list.length; i++)
            servo_list[i] = hardwareMap.get(Servo.class, servo_names.get(i));

        for (int i = 0; i < cr_servo_list.length; i++) {
            cr_servo_list[i] = hardwareMap.get(CRServo.class, cr_servo_names.get(i));
            dc_motor_list[i].setDirection(invert_cr_servos[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        telemetry.addData("Robot Hardware", "Initialized");
        telemetry.update();

        map = hardwareMap;
    }

    //IMU Stuff
    public static double getAngle() {
        if (use_IMU) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS);
            return (angles.firstAngle - imu_zero) * (invertIMU ? -1 : 1);
        }
        return 0;
    }

    //Voltage
    public static double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : map.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    //Distance Sensor
    public static double getDistInch(String name){
        return distance_sensor_list[distance_sensor_names.indexOf(name)].getDistance(DistanceUnit.INCH);
    }

    //Touch Sensor
    public static boolean touchSensorTouching(String name) {
        return touch_sensor_list[touch_sensor_names.indexOf(name)].isPressed();
    }

    //Color Sensor
    public static int[] getRGBA(String name) {
        return new int[] {
                color_sensor_list[color_sensor_names.indexOf(name)].red(),
                color_sensor_list[color_sensor_names.indexOf(name)].green(),
                color_sensor_list[color_sensor_names.indexOf(name)].blue(),
                color_sensor_list[color_sensor_names.indexOf(name)].alpha()
        };
    }

    //LED
    public static void setLed(String name, String pattern) {
        RevBlinkinLedDriver.BlinkinPattern convertedPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        switch (pattern.toLowerCase()) {
            case "rainbow":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                break;
            case "red":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                break;
            case "orange":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                break;
            case "yellow":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case "green":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case "blue":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                break;
            case "violet":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case "purple":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case "white":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            case "black":
                convertedPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                break;
        }
        led_list[led_names.indexOf(name)].setPattern(convertedPattern);
    }
}

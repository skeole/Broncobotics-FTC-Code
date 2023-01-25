package org.firstinspires.ftc.teamcode.Autonomous.Team202;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robots.*;

@Autonomous(name = "Autonomous 202")
public class Auton202 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DoubleArm bhajfiwle = new DoubleArm();
        bhajfiwle.init(hardwareMap);
        bhajfiwle.start();
        sleep(2000);
        bhajfiwle.set_height(1.5);
        telemetry.addData("see, it's in a", "thread");
        telemetry.update();
        sleep(4000);
        bhajfiwle.set_height(0.5);
        telemetry.addData("you can also see this because", "the pid is running");
        telemetry.update();
        sleep(4000);
        bhajfiwle.set_height(-1); // we will have to figure stuff out, or something, i think we can just not reset the encoder values in teleop
        sleep(2000);
    }
}

class DoubleArm extends Thread {

    private static DcMotor right_motor, left_motor, joint; // left_motor just follows right_motor
    private static double right_motor_target = 0.0, joint_target = 0.0;
    private static final double first_arm_zero = 410, second_arm_zero = -817, ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI;

    private static Servo wrist;
    private static double wrist_target = 0.0;

    private static boolean should_be_running = true;

    public static void init(HardwareMap map) {
        right_motor = map.get(DcMotor.class, "joint1right");
        left_motor = map.get(DcMotor.class, "joint1left");
        joint = map.get(DcMotor.class, "joint2");
        wrist = map.get(Servo.class, "clawAligner");
        set_position(1, 0);
    }

    public static void set_position(double x, double y) {
        if (Math.abs(x * x + y * y) > 3.9) {
            return; // will throw an error
        }
        double magnitude = Math.sqrt(x * x + y * y);

        double temp_angle = Math.atan(y / x);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two;
        // removing the initial angle

        target_angle_one *= ticks_per_radian;
        target_angle_two *= ticks_per_radian;

        target_angle_three *= 6.0 / (5.0 * Math.PI);
        // 300 is the maximum angle --> 5/6 pi --> sets target angle to 1

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * 0.00334 + 0.33577;

        right_motor_target = target_angle_one;

        joint_target = target_angle_two;

        wrist_target = target_angle_three;
    }

    public static void set_height(double y) {
        set_position(1, y);
    }

    public static void quit() {
        should_be_running = false;
    }

    public void run() {
        while (should_be_running) {
            right_motor.setPower(Math.max(Math.min(0.02 * (right_motor_target - right_motor.getCurrentPosition()), max_power[0]), min_power[0]));
            left_motor.setPower(right_motor.getPower());

            joint.setPower(Math.max(Math.min(0.02 * (joint_target - joint.getCurrentPosition()), max_power[1]), min_power[1]));

            wrist.setPosition(wrist_target);
        }
    }
}
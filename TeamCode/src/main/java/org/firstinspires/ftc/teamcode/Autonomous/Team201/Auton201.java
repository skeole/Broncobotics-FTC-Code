package org.firstinspires.ftc.teamcode.Autonomous.Team201;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Robots.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.PositionControl;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.Tensorflow;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.ThreadedMotor;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.ThreadedServo;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogicBase;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.RobotHardware;
import org.firstinspires.ftc.teamcode.Robots;


class Auton201Logic extends Thread {

    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;

    public static boolean isRunning = true;

    public static DR4B double_reverse = new DR4B();

    public void run() { // MAIN FUNCTION

        double_reverse.start();

        pause(2);

        DR4B.set_position(4);

        telemetry.addData("see, it's in a", "thread");

        telemetry.update();
        pause(4);
        DR4B.move_virtual_up();

        pause(2);
        DR4B.stop_virtual();
        DR4B.set_position(2);

        telemetry.addData("you can also see this because", "the pid is running");
        telemetry.update();

        pause(4);

        DR4B.set_position(0);

        pause(2000);

        DR4B.quit();
        isRunning = false;
    }

    public static void quit() { // deactivate all threads
        DR4B.quit();
        AutonomousLogicBase.stop();
    }

    public static void init(HardwareMap map, Telemetry telem) {
        hardwareMap = map;
        telemetry = telem;
        DR4B.init(hardwareMap);

        AutonomousLogicBase.init201();
        AutonomousLogicBase.initialize_hardware(hardwareMap, telemetry);

        AutonomousLogicBase.resetEncoder("Left");
        AutonomousLogicBase.resetEncoder("Right");
    }

    public static void pause(double seconds) {
        long t = System.nanoTime();
        while (System.nanoTime() - t <= seconds * 1000000000.0) {
        }
    }
}

@Autonomous(name = "Autonomous 201")
public class Auton201 extends LinearOpMode {

    Auton201Logic logic;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        logic.start();

        while (opModeIsActive() && Auton201Logic.isRunning) {
            idle();
        }

        Auton201Logic.quit();
        stop();
    }
}

class DR4B extends Thread { // aka motor handler

    public static final int[] heights = {
            0,
            100,
            300,
            600,
            1100
    };

    private static DcMotor right_motor, left_motor; // left_motor just follows right_motor
    private static int target_height = 0;
    private static double claw_target = 0.2;
    private static double virtual_power = 0.2;

    private static Servo claw;

    private static CRServo virtual;

    private static boolean should_be_running = true;

    public static void init(HardwareMap map) {
        left_motor = map.get(DcMotor.class, "Left");
        right_motor = map.get(DcMotor.class, "Right");
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_motor.setDirection(DcMotor.Direction.REVERSE);

        claw = map.get(Servo.class, "Scissor");
        virtual = map.get(CRServo.class, "Virtual");
    }

    public static void set_position(int index) {
        target_height = heights[index];
    }

    public static void open_claw() {
        claw_target = 0.6;
    }

    public static void close_claw() {
        claw_target = 0.2;
    }

    public static void move_virtual_up() {
        virtual_power = 0.9;
    }

    public static void move_virtual_down() {
        virtual_power = -0.9;
    }

    public static void stop_virtual() {
        virtual_power = 0;
    }

    public static void quit() {
        should_be_running = false;
    }

    public void run() {
        close_claw();
        should_be_running = true;
        while (should_be_running) {
            left_motor.setPower(Math.max(Math.min(0.02 * (target_height - left_motor.getCurrentPosition()), max_power[0]), min_power[0]));
            right_motor.setPower(left_motor.getPower());

            claw.setPosition(claw_target);

            virtual.setPower(virtual_power);
        }
    }
}
package org.firstinspires.ftc.teamcode.Autonomous.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.*;
import org.firstinspires.ftc.teamcode.Logic.*;
import org.firstinspires.ftc.teamcode.Robots;

import java.util.HashMap;
import java.util.Map;

class Auton extends Thread {

    public static AutonomousLogicBase alb;
    public static StandardTrackingWheelLocalizer stwl;
    public static Tensorflow tf;

    public static PositionControl robot;
    public static ThreadedMotor motor1;
    public static ThreadedServo servo1;

    public static boolean isRunning = true;

    public void run() { // MAIN FUNCTION
        robot = new PositionControl(stwl, 80.0, 70.0, 270.0);

        motor1 = new ThreadedMotor(RobotHardware.map, "motor name");
        servo1 = new ThreadedServo(RobotHardware.map, "servo name");

        motor1.start();
        servo1.start();

        robot.turn(90.00); // Should turn 90 degrees right
        robot.moveForward(100.0); // Should move 100 degrees forward
        robot.strafe(-100.0); // Should move 100 degrees Left

        pause(3000); // suspend reading of code

        motor1.set_position(300);
        servo1.set_position(0.5);
        AutonomousLogicBase.setPower("motor 2", 0.3);

        waitFor(motor1); //waits for motor1 to finish

        isRunning = false;
    }

    public static void quit() {
        motor1.should_be_running = false;
        servo1.should_be_running = false;
        AutonomousLogicBase.stop();
    }

    public static void init(AutonomousLogicBase r) {
        alb = r;
    }
    public static void init(AutonomousLogicBase r, StandardTrackingWheelLocalizer l) {
        alb = r;
        stwl = l;
    }
    public static void init(AutonomousLogicBase r, Tensorflow t) {
        alb = r;
        tf = t;
    }
    public static void init(AutonomousLogicBase r, StandardTrackingWheelLocalizer l, Tensorflow t) {
        alb = r;
        stwl = l;
        tf = t;
    }

    public static void waitFor(ThreadedMotor motor) {
        while (motor.isBusy) {
        }
    }

    public static void pause(double milliseconds) {
        long t = System.nanoTime();
        while (System.nanoTime() - t <= milliseconds * 1000000) {
        }
    }
}

@Autonomous(name = "Autonomous Example")
public class AutonomousExample extends LinearOpMode {

    AutonomousLogicBase r = new AutonomousLogicBase();
    StandardTrackingWheelLocalizer l;
    Tensorflow t;
    Auton a;

    @Override
    public void runOpMode() throws InterruptedException {
        Robots.init_base();
        RobotHardware.initialize_hardware(hardwareMap, telemetry);
        l = new StandardTrackingWheelLocalizer(hardwareMap, r);
        t = new Tensorflow(r);

        a = new Auton();
        a.init(r, l, t);

        waitForStart();

        a.start();

        while ((opModeIsActive()) && (a.isRunning)) {
            idle();
        }

        throw new IllegalArgumentException("lol");
        //a.quit();
        //stop();
    }

}
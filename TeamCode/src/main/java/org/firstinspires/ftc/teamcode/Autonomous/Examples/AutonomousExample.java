package org.firstinspires.ftc.teamcode.Autonomous.Examples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.*;
import org.firstinspires.ftc.teamcode.Logic.*;

import java.util.HashMap;
import java.util.Map;

class Auton extends Thread {

    AutonomousLogicBase alb;
    StandardTrackingWheelLocalizer stwl;
    Tensorflow tf;

    PositionControl robot;
    ThreadedMotor motor1;
    ThreadedServo servo1;

    UpdatingTelemetry telem;

    public boolean isRunning = true;

    public void run() { //MAIN FUNCTION
        robot = new PositionControl(stwl, 80.0, 70.0, 270.0);
        telem = new UpdatingTelemetry(alb, stwl, tf);

        motor1 = new ThreadedMotor(alb, "motor name");
        servo1 = new ThreadedServo(alb, "servo name");

        motor1.start();
        servo1.start();
        telem.start();

        robot.turn(90.00); //Should turn 90 degrees right
        robot.moveForward(100.0); //Should move 100 degrees forward
        robot.strafe(-100.0); //Should move 100 degrees Left

        telem.add_element_to_telemetry("this", "is how you add data to telemetry");

        pause(3000); //suspend reading of code

        telem.remove("this"); //is how you remove data from the telemetry
        telem.show_tensor = false;
        telem.add_element_to_telemetry("no longer showing", "tensorflow data");

        motor1.set_position(300);
        servo1.set_position(0.5);
        alb.setPower("motor 2", 0.3);

        waitFor(motor1); //waits for motor1 to finish

        isRunning = false;
    }

    public void quit() {
        motor1.should_be_running = false;
        servo1.should_be_running = false;
        telem.should_be_running = false;
        alb.stop();
    }

    public Auton(AutonomousLogicBase r) {
        alb = r;
    }

    public Auton(AutonomousLogicBase r, StandardTrackingWheelLocalizer l) {
        alb = r;
        stwl = l;
    }

    public Auton(AutonomousLogicBase r, Tensorflow t) {
        alb = r;
        tf = t;
    }

    public Auton(AutonomousLogicBase r, StandardTrackingWheelLocalizer l, Tensorflow t) {
        alb = r;
        stwl = l;
        tf = t;
    }

    public void waitFor(ThreadedMotor motor) {
        while (motor.isBusy) {
        }
    }

    public void pause(double milliseconds) {
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
        r.init201();
        r.initialize_hardware(hardwareMap, telemetry);
        l = new StandardTrackingWheelLocalizer(hardwareMap, r);
        t = new Tensorflow(r);

        a = new Auton(r, l, t);

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

class UpdatingTelemetry extends Thread {
    RobotHardware robot;
    StandardTrackingWheelLocalizer localizer;

    Pose2d currentPose;
    Tensorflow tensorflow;

    HashMap<String, String> telemetry = new HashMap<String, String>();

    public boolean should_be_running = true;
    boolean show_position;
    boolean show_tensor;

    public void add_element_to_telemetry(String key, String value) {
        telemetry.put(key, value);
    }

    public void remove(String key) {
        telemetry.remove(key);
    }

    public void run() {
        while (should_be_running) {
            localizer.update();
            currentPose = localizer.getPoseEstimate();
            robot.telemetry.clear();
            if (show_position) {
                robot.telemetry.addData("x", currentPose.getX());
                robot.telemetry.addData("y", currentPose.getY());
                robot.telemetry.addData("angle", currentPose.getHeading());
            }
            if (show_tensor) {
                for (String data : tensorflow.getData()) {
                    robot.telemetry.addData("tensorflow detects", data);
                }
            }
            for (Map.Entry<String, String> telem_elem : telemetry.entrySet()) {
                robot.telemetry.addData(telem_elem.getKey(), telem_elem.getValue());
            }
            robot.telemetry.update();
            pause(10);
        }
    }

    public void pause(double milliseconds) {
        long t = System.nanoTime();
        while (System.nanoTime() - t <= milliseconds * 1000000) {
        }
    }

    public UpdatingTelemetry(RobotHardware r) {
        robot = r;
        show_position = false;
        show_tensor = false;
    }

    public UpdatingTelemetry(RobotHardware r, StandardTrackingWheelLocalizer l) {
        robot = r;
        localizer = l;
        localizer.setPoseEstimate(l.getPoseEstimate());
        show_position = true;
        show_tensor = false;
    }

    public UpdatingTelemetry(RobotHardware r, Tensorflow t) {
        robot = r;
        tensorflow = t;
        show_position = false;
        show_tensor = true;
    }

    public UpdatingTelemetry(RobotHardware r, StandardTrackingWheelLocalizer l, Tensorflow t) {
        robot = r;
        localizer = l;
        localizer.setPoseEstimate(l.getPoseEstimate());
        tensorflow = t;
        show_position = true;
        show_tensor = true;
    }
}
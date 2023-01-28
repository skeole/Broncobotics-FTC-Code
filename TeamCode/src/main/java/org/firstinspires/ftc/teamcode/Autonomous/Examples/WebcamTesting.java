package org.firstinspires.ftc.teamcode.Autonomous.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Logic.AutonomousLogic.OpenCV;
import org.firstinspires.ftc.teamcode.Logic.AutonomousLogicBase;

@Autonomous(name = "Webcam testing")
public class WebcamTesting extends LinearOpMode {

    public static final boolean testing = true;

    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousLogicBase alb = new AutonomousLogicBase();
        alb.init202();
        alb.initialize_hardware(hardwareMap, telemetry);
        alb.setPosition("clawAligner", 0.5);
        OpenCV camera = new OpenCV(hardwareMap, telemetry, "Webcam 1");

        sleep(3000); // waiting for timeout to pass

        if (testing) {
            camera.start(); // for some reason the purple side has very low saturation... I don't think its a printing issue
        }

        waitForStart();

        if (!testing) {
            camera.start();
        }

        sleep(30000); // run for 5 minutes
                            // it puts data to the telemetry

        camera.stopStreaming();
    }
}

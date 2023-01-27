package org.firstinspires.ftc.teamcode.Logic;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonomousLogicBase extends RobotHardware {

    public static void wait(double seconds) {
        double start = System.nanoTime() / 1000000000.0;
        while (System.nanoTime() / 1000000000.0 - start < seconds) { /* wait */ }
    }

    public static void stop() {
        for (DcMotor motor : dc_motor_list) {
            motor.setPower(0);
        }
        for (DcMotor motor : wheel_list) {
            motor.setPower(0);
        }
        //alternatives: stop() in LinearOpMode or throw an error
    }

    public static void driveForward(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower(power);
    }

    public static void strafe(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i % 2 == 0 ? 1 : -1) * power);
    }

    public static void turn(double power) {
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i > 1 ? -1 : 1) * power);
    }

    public static void turnDegree(double degrees) {
        double startingAngle = getAngle();

        double targetHeading = startingAngle + degrees;
        while (targetHeading < 0 || targetHeading >= 360) {
            targetHeading += 360 * (targetHeading < 0 ? 1 : -1);
        }

        int factor = (((0 < targetHeading - startingAngle) && (targetHeading - startingAngle < 180)) || (targetHeading - startingAngle < -180) ? 1 : -1);
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower((i < 2 ? -0.5 : 0.5) * factor);

        while ((getAngle() - targetHeading) * (getAngle() - targetHeading) < 25) { /* wait */ }
        for (int i = 0; i < wheel_list.length; i++) wheel_list[i].setPower(0);
    }

    //DC Motors
    public static void setPower(String name, double power) {
        dc_motor_list[dc_motor_names.indexOf(name)].setPower(power);
    }

    public static void resetEncoder(String name) {
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dc_motor_list[dc_motor_names.indexOf(name)].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Servos
    public static void setPosition(String name, double position) {
        servo_list[servo_names.indexOf(name)].setPosition(position);
    }

    //RoadRunner

    public static void initialize_RoadRunner() {}
}
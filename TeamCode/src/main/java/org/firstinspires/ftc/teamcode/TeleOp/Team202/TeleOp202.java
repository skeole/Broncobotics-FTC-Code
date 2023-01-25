package org.firstinspires.ftc.teamcode.TeleOp.Team202;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;
import static org.firstinspires.ftc.teamcode.Robots.*;

class TeleOp202Logic extends TeleOpLogicBase {

    public static double starting_time;

    public static final double x = 1, max_y = Math.sqrt(4 - x * x) - 0.05, lift_speed = 1.3;
    public static double y = 0; // starting y value

    public final static double
            first_arm_zero =     410,
            second_arm_zero =    -817;

    // Encoder Values at straight ahead:
        //410
        //-817

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI; // 2786.2109868741 ticks per revolution

    public static DcMotor left_motor = null;

    public static void execute_non_driver_controlled() {

        if (buttons[4])
            y += delta_time * lift_speed;
        if (buttons[5])
            y -= delta_time * lift_speed;

        y = Math.max(Math.min(y, max_y), 0 - max_y);

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

        target_angle_three *= 180.0 / Math.PI; // angle
            // 300 is the maximum angle --> 5/6 pi --> sets target angle to 1

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * 0.00334 + 0.33577; // very good guessing ig

        dc_target_positions[0] = target_angle_one;
        left_motor.setPower(dc_motor_list[0].getPower());

        dc_target_positions[1] = target_angle_two;

        servo_target_positions[0] = target_angle_three;
        telemetry.addData("position", servo_list[0].getPosition());

        // telemetry.addData("cycles per second", 0.1 * ((int) (10 / delta_time)));
        // telemetry.addData("elapsed time", 0.1 * ((int) (0.5 + 10.0 * (current_time - starting_time))));

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public static void init(HardwareMap hm, Telemetry tm) {
        starting_time = System.nanoTime() / 1000000000.0;
        init202();
        initialize_logic(hm, tm);
        setZeroAngle(0);
        set_keybinds();
        set_button_types();
        left_motor = map.get(DcMotor.class, "joint1left");
        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo_target_positions[0] = 0.6;
    }

    public static void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 0, localizer);
    }

    public static void set_keybinds() {

        new_keybind("clawAligner", "operator right_stick_y", "default", 0.3, 0.3);
        //new_keybind("claw", "operator right_trigger", "default", 0.66, 0.66);
        //new_keybind("claw", "operator dpad_left", "default", "gradient", -0.66);

        // TODO: Figure out why operator left_trigger isn't working for this >:(

    }
}

@TeleOp(name="TeleOp 202", group="Iterative Opmode")
public class TeleOp202 extends LinearOpMode {
    TeleOp202Logic logic = new TeleOp202Logic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}
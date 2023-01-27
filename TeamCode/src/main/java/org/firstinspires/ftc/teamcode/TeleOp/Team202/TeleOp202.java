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

    public static final double x = 1, max_y = Math.sqrt(4 - x * x) - 0.05, min_y = -1, lift_speed = 1.3;
    public static double y = 0; // starting y value

    public final static double
            first_arm_zero =     410,
            second_arm_zero =    -817;

    // Encoder Values at straight ahead:
        // 410
        // -817

    public static final double ticks_per_radian = 2786.2109868741 / 2.0 / Math.PI; // 2786.2109868741 ticks per revolution

    public static DcMotor left_motor = null;

    public static void execute_non_driver_controlled() {

        if (buttons[4])
            y += delta_time * lift_speed;
        if (buttons[5])
            y -= delta_time * lift_speed;

        y = Math.max(Math.min(y, max_y), min_y);

        if (y > 1) {
            max_speed = 0.5;
        } else {
            max_speed = 1;
        }

        double magnitude = Math.sqrt(x * x + y * y);

        double temp_angle = Math.atan(y / x);
        double secondary_angle = Math.acos(magnitude / 2);
        double primary_angle = Math.PI - 2 * secondary_angle;

        double target_angle_one = temp_angle + secondary_angle;
        double target_angle_two = target_angle_one + primary_angle - Math.PI;
        double target_angle_three = 0 - target_angle_two;

        target_angle_one *= ticks_per_radian;
        target_angle_two *= ticks_per_radian;

        target_angle_three *= 180.0 / Math.PI; // angle

        target_angle_one += first_arm_zero;
        target_angle_two += second_arm_zero;
        target_angle_three = target_angle_three * 0.00334 + 0.33577; // very good guessing ig

        dc_target_positions[0] = target_angle_one;
        left_motor.setPower(dc_motor_list[0].getPower());

        dc_target_positions[1] = target_angle_two;

        servo_target_positions[0] = Math.max(Math.min(target_angle_three, servo_max_positions[0]), servo_min_positions[0]);

        telemetry.addData("elapsed time", 0.1 * ((int) (0.5 + 10.0 * (current_time - starting_time))));

        telemetry.addData("operator left trigger", operator_left_trigger);

        telemetry.addData("claw target", servo_target_positions[1]);

        //debugging code
        for (int i = 0; i < dc_motor_list.length; i++) {
            telemetry.addData(dc_motor_names.get(i), dc_motor_list[i].getCurrentPosition());
        }

        telemetry.addLine();

        for (int i = 0; i < servo_list.length; i++) {
            telemetry.addData(servo_names.get(i), servo_list[i].getPosition());
        }

        telemetry.addLine();

        for (int i = 0; i < keys.size(); i++) {
            if ((i < 20) && buttons[i]) {
                telemetry.addData(keys.get(i), "is pressed");
            } else if ((i > 19) && Math.abs(axes[i - 20]) > 0.1) {
                telemetry.addData(keys.get(i) + " value", axes[i - 20]);
            }
        }
        telemetry.addData("y", y);

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
        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo_target_positions[0] = 0.6;
        servo_target_positions[1] = 0.5;
    }

    public static void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 0, localizer);
    }

    public static void set_keybinds() {

        new_keybind("claw", "operator right_trigger", "default", 0.66, 0.66);
        new_keybind("claw", "operator left_trigger", "default", -0.66, -0.66);
        // so it is being pressed ifk

        // new_keybind("claw", "operator a", "cycle", 1, 0); // maybe this?

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
            telemetry.addData("operator left trigger actual", gamepad2.left_trigger);
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}
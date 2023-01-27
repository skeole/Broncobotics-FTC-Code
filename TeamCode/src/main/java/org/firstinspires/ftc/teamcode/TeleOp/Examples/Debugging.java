package org.firstinspires.ftc.teamcode.TeleOp.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;
import static org.firstinspires.ftc.teamcode.Robots.*;

class DebuggingLogic extends TeleOpLogicBase {

    public static double starting_time;

    public static void execute_non_driver_controlled() {

        if (useRoadRunner) {
            telemetry.addData("target x", target_x);
            telemetry.addData("target y", target_y);

            telemetry.addData("current x", current_x);
            telemetry.addData("current y", current_y);
        }

        if ((useRoadRunner) || (usePID)) {
            telemetry.addData("target angle", target_angle);
            telemetry.addData("current angle", current_angle);

            telemetry.addData("angle to field", angle());
        }

        telemetry.addData("cycles per second", 0.1 * ((int) (10 / delta_time)));
        telemetry.addData("elapsed time", 0.1 * ((int) (10.0 * (starting_time - current_time))));

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

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    public static void init(HardwareMap hm, Telemetry tm) {
        starting_time = System.nanoTime() / 1000000000.0;
        init_base();
        initialize_logic(hm, tm);
        setZeroAngle(0);
        set_keybinds();
        set_button_types();
    }

    public static void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public static void set_keybinds() {

    }
}

@TeleOp(name="Debugging", group="Iterative Opmode")
public class Debugging extends LinearOpMode {
    DebuggingLogic logic = new DebuggingLogic();
    StandardTrackingWheelLocalizer localizer;
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.execute_non_driver_controlled();
        }
    }
}
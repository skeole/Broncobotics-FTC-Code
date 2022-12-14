package org.firstinspires.ftc.teamcode.TeleOp.Examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;

class DriverPracticeLogic extends TeleOpLogicBase {

    public void execute_non_driver_controlled() {

        if (useRoadRunner) {
            telemetry.addData("target x", target_x);
            telemetry.addData("target y", target_y);

            telemetry.addData("current x", current_x);
            telemetry.addData("current y", current_y);
        }

        if ((useRoadRunner) || (usePID)) {
            telemetry.addData("target angle", target_angle);
            telemetry.addData("current angle", current_angle);
        }

        telemetry.addData("angle to field", angle());

        if (buttons[keys.indexOf("driver a")]) {
            telemetry.addData("resetting", "imu angle");
            resetZeroAngle();
        }

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }


    public void init(HardwareMap hm, Telemetry tm) {
        init201();
        initialize_hardware(hm, tm);
        setZeroAngle(0);
        button_types[keys.indexOf("driver a")] = 3; //1 = default, 2 = toggle, 3 = button
    }

    public void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 90, localizer);
    }

    public void set_keybinds() {

    }

    public DriverPracticeLogic() {
        super();
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="Driver Practice", group="Iterative Opmode")
public class DriverPractice extends LinearOpMode {
    DriverPracticeLogic logic = new DriverPracticeLogic();
    StandardTrackingWheelLocalizer localizer;
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (logic.useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2); //driver is gamepad1, operator is gamepad2
            logic.execute_non_driver_controlled();
        }
    }
}
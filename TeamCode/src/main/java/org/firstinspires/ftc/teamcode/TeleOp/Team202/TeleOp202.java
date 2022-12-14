package org.firstinspires.ftc.teamcode.TeleOp.Team202;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Logic.TeleOpLogicBase;

class TeleOp202Logic extends TeleOpLogicBase {

    public void execute_non_driver_controlled() {

        telemetry.addData("Angle?", getAngle());
        telemetry.addData("Angle V2", current_angle = 0 - getAngle() - zero_angle); //Only different value if not starting robot straight ahead

        telemetry.update();

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init(HardwareMap hm, Telemetry tm) {
        init202();
        initialize_hardware(hm, tm);
        setZeroAngle(0);

    }

    public void initRoadRunner(StandardTrackingWheelLocalizer localizer) {
        initializeRoadRunner(45, 100, 0, localizer);
    }

    public void set_keybinds() {
    }

    public TeleOp202Logic() {
        super();
        set_keybinds();
        set_button_types();
    }
}

@TeleOp(name="TeleOp 202", group="Iterative Opmode")
public class TeleOp202 extends LinearOpMode {
    TeleOp202Logic logic = new TeleOp202Logic();
    @Override
    public void runOpMode() throws InterruptedException {
        logic.init(hardwareMap, telemetry);
        waitForStart();
        if (logic.useRoadRunner) {
            logic.initRoadRunner(new StandardTrackingWheelLocalizer(hardwareMap, logic));
        }
        while (opModeIsActive()) {
            logic.tick(gamepad1, gamepad2);
            logic.execute_non_driver_controlled();
        }
    }
}
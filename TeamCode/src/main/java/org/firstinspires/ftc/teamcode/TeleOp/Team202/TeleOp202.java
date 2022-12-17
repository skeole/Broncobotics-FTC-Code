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

    public double starting_time;

    double tx = 2.0;
    double ty = 0.0;

    boolean clawOpen = false;

    static double CLAW_OPEN = 1;
    static double CLAW_CLOSE = 0;

    double CLAW_ALIGNER_INCREMENTER = 10;

    double z1 = Math.PI / 180.0 * 5;
    double z2 = Math.PI / 180.0 * 354;

    double tpr = 2786.2109868741 / 2.0 / Math.PI;

    DcMotor dc = null;

    public void execute_non_driver_controlled() {

        double xbefore = tx;
        double ybefore = ty;

        if (buttons[4])
            ty += delta_time * 2;
        if (buttons[5])
            ty -= delta_time * 2;
        if (buttons[7])
            tx += delta_time * 2;
        if (buttons[6])
            tx -= delta_time * 2;
        if (buttons[9]) {
            clawOpen = true;
            servo_target_positions[3] = CLAW_OPEN;
        }
        if (buttons[8]) {
            clawOpen = false;
            servo_target_positions[3] = CLAW_CLOSE;
        }

        ty += axes[2] * CLAW_ALIGNER_INCREMENTER * delta_time;
        tx += axes[0] * CLAW_ALIGNER_INCREMENTER * delta_time;

        if (ty < Math.sqrt(3)) {
            tx = 1;
            if (ty < -1.2) ty = -1.2;
        } else {
            tx = Math.sqrt(4 - ty * ty);
        }

        if (tx <= 0.001) tx = 0.001;

        if (buttons[3]) {
            tx = 1;
            ty = 1.7;
        }

        if (buttons[2]) {
            ty = -1.2;
            tx = 1.2;
        }
        if (buttons[0]) {
            ty = 1.8;
            tx = 0.8;
        }
        if (ty >= 2) ty = 2;
        if (tx >= 2) tx = 2;
        if (ty < -1.2) ty = -1.2;
        if (tx < 0.5) tx = 0.5;
        double magnitude = Math.sqrt(tx * tx + ty * ty);

        if (magnitude > 2) {
            double ong = 1.995 / magnitude;
            double ratio = ong;
            tx *= ratio;
            ty *= ong;
            magnitude = Math.sqrt(tx * tx + ty * ty);
        }

        double tangle2 = Math.acos(1 - magnitude * magnitude / 2.0); //180 means straight line
        double tangle1 = Math.PI + Math.atan(ty / tx) - tangle2 / 2.0; //0 means straight down
        double tangle3 = Math.PI / 2.0 - tangle2 - tangle1;

        // removing the initial angle
        tangle1 -= z1;
        tangle2 -= z2;

        // converting to encoder ticks
        tangle1 *= tpr;
        tangle2 *= tpr;

        dc_target_positions[0] = 0 - tangle1; //ticks per radian
        dc.setPower(dc_motor_list[0].getPower());


        dc_target_positions[1] = tangle2;

        if (axes[4] > 0.1) {
            servo_target_positions[0]+= axes[4] * CLAW_ALIGNER_INCREMENTER;
        }
        if (axes[5] > 0.1) {
            servo_target_positions[0]-= axes[5] * CLAW_ALIGNER_INCREMENTER;
        }

        telemetry.addData("Angle?", getAngle());
        telemetry.addData("Angle V2", current_angle = 0 - getAngle() - zero_angle); //Only different value if not starting robot straight ahead

        telemetry.addData("cycles per second", 0.1 * ((int) (10 / delta_time)));
        telemetry.addData("elapsed time", 0.1 * ((int) (10.0 * (starting_time - current_time))));

        telemetry.update();

        telemetry.update();
        if (useRoadRunner) {
            position_tracker.update();
        }
    }

    //Initialization

    public void init(HardwareMap hm, Telemetry tm) {
        starting_time = System.nanoTime() / 1000000000.0;
        init202();
        initialize_logic(hm, tm);
        dc = map.get(DcMotor.class, "joint1left");
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
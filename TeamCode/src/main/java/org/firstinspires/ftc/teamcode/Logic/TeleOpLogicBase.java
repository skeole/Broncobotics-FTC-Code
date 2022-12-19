package org.firstinspires.ftc.teamcode.Logic;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.nullness.qual.RequiresNonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Logic.RoadRunner.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.InputMismatchException;
import java.util.Map;

public class TeleOpLogicBase extends RobotHardware {
    //run .init201() then .init()
    double previous_time = System.nanoTime() / 1000000000.0;
    public double current_time = System.nanoTime() / 1000000000.0;
    public double delta_time = 0;

    public StandardTrackingWheelLocalizer position_tracker;

    public HashMap<String, ArrayList<Integer>> keybinds = new HashMap<>();
        /* Format
            Motor 1 : button, modifier1, modifier2, modifier3, button, modifier1, modifier2, modifier3
            Servo 1 : button, modifier1, modifier2, modifier3

            for buttons: save index

            For Motors/Non-CR Servos:
                for modifier1: default = 1, toggle = 2, button = 3, cycle = 4
                for modifier2: normal = 0, gradient = 1, power is integer of power * 100
                for modifier3: power is integer of power * 100, or index of cycle we use
            For CRServos:
                for modifier1: default = 1, toggle = 2, button = 3, cycle = 4
                for modifier2: normal = 0, gradient = 1, power is integer of power * 100
                for modifier3: power is integer of power * 100
            For GoTo:
                for modifier1: x * 1000
                for modifier2: y * 1000
                for modifier3: angle * 1000 (for angle to remain the same set angle to precisely 1 because ain't no way I set angle to 0.001)
         */

    public double[] dc_times_started; //in seconds
    public double[] dc_target_positions;
    public double[][] error; //1st is previous, 2nd is current

    public double[] servo_times_started;
    public double[] servo_target_positions;

    public double[] cr_times_started;

    public int[] button_types = new int[27]; //1 = default, 2 = toggle, 3 = button
    public int[] key_values = new int[27]; //number of times button/axis is "activated"
    public boolean[] buttons = new boolean[20]; //value of button (True or False)
    public double[] axes = new double[7]; //value of axis (1 for buttons/cycles, -1.0 to 1.0 for everything else)

    public double current_x = 0;
    public double current_y = 0;
    public double current_angle = 0;

    public double target_x = 0;
    public double target_y = 0;
    public double target_angle = 0;

    public double zero_angle = 0;

    public double current_error;
    public double previous_error;

    public TeleOpLogicBase() {
        for (String servo : servo_names) keybinds.put(servo, new ArrayList<>());
        for (String motor : dc_motor_names) keybinds.put(motor, new ArrayList<>());
        for (String cr_servo : cr_servo_names) keybinds.put(cr_servo, new ArrayList<>());
        keybinds.put("goto", new ArrayList<>());

        if ((useRoadRunner) && (usePID)) throw new IllegalArgumentException("You cannot use both RoadRunner and the build-in PID");
        if (((locked_motion) || (locked_rotation)) && !((useRoadRunner) || (usePID))) throw new IllegalArgumentException("You can't use locked motion or rotion without a PID method");
    }

    public void initialize_logic(HardwareMap hardwareMap, Telemetry telemetry) {
        initialize_hardware(hardwareMap, telemetry);

        dc_times_started = new double[dc_motor_names.size()]; //in seconds
        dc_target_positions = new double[dc_motor_names.size()];
        error = new double[dc_motor_names.size()][2]; //1st is previous, 2nd is current

        servo_times_started = new double[servo_names.size()];
        servo_target_positions = new double[servo_names.size()];

        cr_times_started = new double[cr_servo_names.size()];
    }

    public void tick(Gamepad gamepad1, Gamepad gamepad2) {
        previous_time = current_time;
        current_time = System.nanoTime() / 1000000000.0;
        delta_time = current_time - previous_time;
        if (delta_time <= 0) delta_time = 0.001;
        update_buttons(gamepad1, gamepad2);
        drive(gamepad1);
        update_robot();
    }

    public void update_button(boolean button_pressed, int button_index) {
        boolean button_active;
        if (button_types[button_index] == 2) { //toggle
            key_values[button_index] += (button_pressed == (key_values[button_index] % 2 == 0)) ? 1 : 0;
            button_active = (key_values[button_index] % 4 != 0);
        } else if (button_types[button_index] == 1) { //default
            button_active = button_pressed;
        } else { //undeclared or button
            button_active = (key_values[button_index] % 2 == 0) && (button_pressed);
            key_values[button_index] += (button_pressed == (key_values[button_index] % 2 == 0)) ? 1 : 0;
        }
        buttons[button_index] = button_active;
    }

    public void update_axis(double axis, int axis_index) {
        double axis_value;
        if (button_types[axis_index] == 2) {
            key_values[axis_index] += ((Math.abs(axis) > 0.1) == (key_values[axis_index] % 2 == 0)) ? 1 : 0;
            axis_value = key_values[axis_index] % 4 != 0 ? 1 : 0;
        } else if (button_types[axis_index] == 1) {
            axis_value = axis;
        } else {
            axis_value = (key_values[axis_index] % 2 == 0) && (Math.abs(axis) > 0.1) ? 1 : 0;
            key_values[axis_index] += ((Math.abs(axis) > 0.1) == (key_values[axis_index] % 2 == 0)) ? 1 : 0;
        }
        axes[axis_index - 20] = axis_value;
    }

    public void update_buttons(Gamepad gamepad1, Gamepad gamepad2) {
        update_button(gamepad2.a, 0);
        update_button(gamepad2.b, 1);
        update_button(gamepad2.x, 2);
        update_button(gamepad2.y, 3);
        update_button(gamepad2.dpad_up, 4);
        update_button(gamepad2.dpad_down, 5);
        update_button(gamepad2.dpad_left, 6);
        update_button(gamepad2.dpad_right, 7);
        update_button(gamepad2.left_bumper, 8);
        update_button(gamepad2.right_bumper, 9);

        update_button(gamepad1.a, 10);
        update_button(gamepad1.b, 11);
        update_button(gamepad1.x, 12);
        update_button(gamepad1.y, 13);
        update_button(gamepad1.dpad_up, 14);
        update_button(gamepad1.dpad_down, 15);
        update_button(gamepad1.dpad_left, 16);
        update_button(gamepad1.dpad_right, 17);
        update_button(gamepad1.left_bumper, 18);
        update_button(gamepad1.right_bumper, 19);

        update_axis(gamepad2.left_stick_x, 20);
        update_axis(gamepad2.right_stick_x, 21);
        update_axis(0 - gamepad2.left_stick_y, 22); //negative because down means positive for FTC controllers
        update_axis(0 - gamepad2.right_stick_y, 23);
        update_axis(gamepad2.left_trigger, 24);
        update_axis(gamepad2.right_trigger, 25);
        update_axis(gamepad1.left_trigger, 26);
    }

    public void drive(Gamepad gamepad) {
        double speedFactor = 1 + 2 * gamepad.right_trigger;

        double left_stick_magnitude = Math.sqrt(gamepad.left_stick_x * gamepad.left_stick_x + gamepad.left_stick_y * gamepad.left_stick_y);
        if (left_stick_magnitude <= 0.333) left_stick_magnitude = 0.0;
        double left_stick_angle =
            (left_stick_magnitude <= 0.333) ? -Math.PI / 2.0 :
            (gamepad.left_stick_x > 0) ? Math.atan(gamepad.left_stick_y/gamepad.left_stick_x) :
            (gamepad.left_stick_x < 0) ? Math.PI + Math.atan(gamepad.left_stick_y/gamepad.left_stick_x) :
            (gamepad.left_stick_y > 0) ? Math.PI / 2.0 : -Math.PI / 2.0;
        left_stick_angle += Math.PI/2.0;

        double right_stick_magnitude = Math.sqrt(gamepad.right_stick_x * gamepad.right_stick_x + gamepad.right_stick_y * gamepad.right_stick_y);
        if (right_stick_magnitude <= 0.333) right_stick_magnitude = 0.0;
        double right_stick_angle =
            (right_stick_magnitude <= 0.333) ? -Math.PI / 2.0 :
            (gamepad.right_stick_x > 0) ? Math.atan(gamepad.right_stick_y/gamepad.right_stick_x) :
            (gamepad.right_stick_x < 0) ? Math.PI + Math.atan(gamepad.right_stick_y/gamepad.right_stick_x) :
            (gamepad.right_stick_y > 0) ? Math.PI / 2.0 : -Math.PI / 2.0;
        right_stick_angle += Math.PI/2.0;

        left_stick_angle = modifiedAngle(left_stick_angle);
        right_stick_angle = modifiedAngle(right_stick_angle);

        //Positive angles --> clockwise
        //Zero --> vertical

        if (usePID) {
            current_angle = 0 - getAngle() - zero_angle;
        } else if (useRoadRunner) {
            position_tracker.update();
            Pose2d currentPose = position_tracker.getPoseEstimate();

            current_x = currentPose.getX();
            current_y = currentPose.getY();

            current_angle = 0 - currentPose.getHeading() - zero_angle;
        } //current_angle: same angle system as left/right stick angle

        double distance_factor;
        double offset;

        if (left_stick_magnitude != 0) {
            distance_factor = left_stick_magnitude;

            if (locked_motion) {
                offset = modifiedAngle(left_stick_angle - current_angle);
            } else {
                offset = left_stick_angle;
            }

            if (useRoadRunner) {
                target_x = current_x;
                target_y = current_y;
            }

        } else {

            distance_factor = Math.sqrt((current_x - target_x) * (current_x - target_x) + (current_y - target_y) * (current_y - target_y)) * distance_weight_two;
            //zero by default if not using RoadRunner :)

            double line_angle = (target_x > current_x) ? Math.atan(((float) target_y - (float) current_y)/((float) target_x - (float) current_x)) :
                (target_x < current_x) ? Math.PI + Math.atan(((float) target_y - (float) current_y)/((float) target_x - (float) current_x)) :
                (target_y > current_y) ? Math.PI / 2.0 : -Math.PI / 2.0;

            line_angle += Math.PI / 2.0;
            offset = modifiedAngle(line_angle - current_angle);
        }

        double turning_factor = 0;

        if (right_stick_magnitude != 0) {
            if (locked_rotation) {
                target_angle = right_stick_angle;
            } else { //if we're driving normally
                target_angle = current_angle;
                turning_factor = gamepad.right_stick_x;
            }
        } //target angle remains constant if we aren't turning manually

        drive(turning_factor, distance_factor, offset, speedFactor);
    }

    public void drive(double turning_factor, double distance_factor, double offset, double speedFactor) {
        double correction = getCorrection();
        turning_factor *= turning_weight;
        turning_factor += correction;
        distance_factor *= distance_weight;
        double[] power = new double[4];
        for (int i = 0; i < 4; i++) {
            power[i] = turning_factor * ((i > 1) ? -1 : 1) - distance_factor * (Math.cos(offset) + Math.sin(offset) * (i % 2 == 1 ? 1 : -1) / strafe);
        }
        double maximum = Math.max(1, Math.max(Math.max(Math.abs(power[0]), Math.abs(power[1])), Math.max(Math.abs(power[2]), Math.abs(power[3]))));
        for (int i = 0; i < 4; i++) {
            wheel_list[i].setPower(max_speed * power[i] / maximum / speedFactor);
        }
    }

    public double getCorrection() {
        current_error = modifiedAngle(target_angle - current_angle);

        double p = current_error;
        double d = (current_error - previous_error) / (current_time - previous_time);

        previous_error = current_error;

        return p_weight * p - d_weight * d;
    }

    public double modifiedAngle(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < 0 - Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public void update_robot() { //The mother of all methods

        for (Map.Entry<String, ArrayList<Integer>> element : keybinds.entrySet()) { //for every element in keybinds

            ArrayList<Integer> object_keys = element.getValue(); //object_keys = what the motor maps to

            int number_of_keys = object_keys.size() / 4; //number of keys that map to the motor
            boolean object_is_active = false; //object is active iff at least one key that maps to it is activated

            int key_index;
            int list_index;

            for (int i = 0; i < number_of_keys; i += 4) {
                key_index = object_keys.get(i);
                object_is_active = ((object_is_active) || ((key_index < 20) && (buttons[key_index])) || ((key_index > 19) && (Math.abs(axes[key_index - 20]) > 0.1)));
            }

            if (dc_motor_names.contains(element.getKey())) {
                list_index = dc_motor_names.indexOf(element.getKey());

                if (!object_is_active) {
                    error[list_index][0] = error[list_index][1];
                    error[list_index][1] = dc_target_positions[list_index] - dc_motor_list[list_index].getCurrentPosition();
                    double derivative = (error[list_index][1] - error[list_index][0]) / delta_time;
                    if (error[list_index][1] > 0) { //current < target --> positive power
                        dc_motor_list[list_index].setPower(Math.max(Math.min(
                            error[list_index][1] * p_weights[list_index] - derivative * d_weights[list_index],
                            max_power[list_index]), 0));
                    } else { //negative
                        dc_motor_list[list_index].setPower(Math.min(Math.max(
                                error[list_index][1] * p_weights[list_index] - derivative * d_weights[list_index],
                                min_power[list_index]), 0));
                    }

                    dc_times_started[list_index] = -10.0;
                } else {

                    if (dc_times_started[list_index] < 0) dc_times_started[list_index] = current_time;

                    error[list_index][0] = 0;
                    error[list_index][1] = 0;

                    for (int i = 0; i < number_of_keys; i += 4) {
                        key_index = object_keys.get(i);
                        if (((key_index < 20) && (buttons[key_index])) || ((key_index > 19) && (axes[key_index - 20] > 0.1))) {
                            //if the key is active
                            if (object_keys.get(i + 1) > 2) { //3 --> button, 4 --> cycle
                                double[] positions_list = positions[object_keys.get(i + 3)];
                                if (positions_list.length == 1) {
                                    dc_target_positions[key_index] = positions_list[0];
                                } else {
                                    boolean increasing = (positions_list[1] > positions_list[0]);
                                    int current_index = 0;
                                    while ((current_index < positions_list.length) && ((positions_list[current_index] < dc_target_positions[list_index]) || (!increasing)) && ((positions_list[current_index] > dc_target_positions[list_index]) || (increasing))) {
                                        current_index += 1;
                                    }
                                    if ((object_keys.get(i + 2) > 0) && ((current_index + 1 > positions_list.length)  || (positions_list[current_index] != dc_target_positions[key_index]))) {
                                        current_index -= 1;
                                    }
                                    if (object_keys.get(i + 1) == 4) { //if cycle
                                        if ((current_index + 2 > positions_list.length) && (object_keys.get(i + 2) > 0)) {
                                            current_index = 0;
                                        } else if ((current_index < 1) && (object_keys.get(i + 2) < 0)) {
                                            current_index = positions_list.length - 1;
                                        } else {
                                            current_index = Math.max(Math.min(current_index + object_keys.get(i + 2), positions.length - 1), 0);
                                        }
                                    } else {
                                        current_index = Math.max(Math.min(current_index + object_keys.get(i + 2), positions.length - 1), 0);
                                    }
                                    dc_target_positions[list_index] = positions_list[current_index];
                                }
                            } else {
                                dc_target_positions[list_index] = Math.max(Math.min(
                                        dc_motor_list[list_index].getCurrentPosition(),
                                motor_max_positions[list_index]), motor_min_positions[list_index]);

                                double calculated_power;

                                if ((object_keys.get(i + 1) == 2) || (key_index < 20)) {
                                    calculated_power = 0.01 * object_keys.get(i + 3) * (1 - object_keys.get(i + 2)) + object_keys.get(i + 2) * Math.min(1, (current_time - dc_times_started[list_index]) / 0.75);
                                                                            //if normal it's 0; if gradient it's 1 >:)
                                } else { //if default axis
                                    calculated_power = axes[key_index - 20] * 0.01 * ( //similar to button defaults, except no gradient option
                                        (key_index > 23) ? object_keys.get(i + 2) : //if it's a trigger, then set it to the first val
                                        (axes[key_index - 20] < 0 ? object_keys.get(i + 2) : object_keys.get(i + 3))
                                    );
                                }

                                calculated_power = Math.max(min_power[list_index], Math.min(max_power[list_index], calculated_power));
                                if ((dc_motor_list[list_index].getCurrentPosition() > motor_max_positions[list_index]) && (calculated_power > 0)) {
                                    calculated_power = Math.max(min_power[list_index], (
                                        motor_max_positions[list_index] - dc_motor_list[list_index].getCurrentPosition()) * p_weights[list_index]);
                                } else if (dc_motor_list[list_index].getCurrentPosition() < motor_min_positions[list_index] && (calculated_power < 0)) {
                                    calculated_power = Math.min(max_power[list_index],
                                        (motor_min_positions[list_index] - dc_motor_list[list_index].getCurrentPosition()) * p_weights[list_index]);
                                }
                                dc_motor_list[list_index].setPower(calculated_power);
                            }
                        }
                    }
                }
            } else if (servo_names.contains(element.getKey())) {
                list_index = servo_names.indexOf(element.getKey());

                if (!object_is_active) {
                    servo_list[list_index].setPosition(servo_target_positions[list_index]);

                    servo_times_started[list_index] = -10.0;
                } else {

                    if (servo_times_started[list_index] < 0) servo_times_started[list_index] = current_time;

                    for (int i = 0; i < number_of_keys; i += 4) {
                        key_index = object_keys.get(i);
                        if (((key_index < 20) && (buttons[key_index])) || ((key_index > 19) && (axes[key_index - 20] > 0.1))) {
                            //if the key is active
                            if (object_keys.get(i + 1) > 2) { //3 --> button, 4 --> cycle
                                double[] positions_list = positions[object_keys.get(i + 3)];
                                if (positions_list.length == 1) {
                                    servo_target_positions[key_index] = positions_list[0];
                                } else {
                                    boolean increasing = (positions_list[1] > positions_list[0]);
                                    int current_index = 0;
                                    while ((current_index < positions_list.length) && ((positions_list[current_index] < servo_target_positions[list_index]) || (!increasing)) && ((positions_list[current_index] > servo_target_positions[list_index]) || (increasing))) {
                                        current_index += 1;
                                    }
                                    if ((object_keys.get(i + 2) > 0) && ((current_index + 1 > positions_list.length)  || (positions_list[current_index] != servo_target_positions[key_index]))) {
                                        current_index -= 1;
                                    }
                                    if (object_keys.get(i + 1) == 4) { //if cycle
                                        if ((current_index + 2 > positions_list.length) && (object_keys.get(i + 2) > 0)) {
                                            current_index = 0;
                                        } else if ((current_index < 1) && (object_keys.get(i + 2) < 0)) {
                                            current_index = positions_list.length - 1;
                                        } else {
                                            current_index = Math.max(Math.min(current_index + object_keys.get(i + 2), positions.length - 1), 0);
                                        }
                                    } else {
                                        current_index = Math.max(Math.min(current_index + object_keys.get(i + 2), positions.length - 1), 0);
                                    }
                                    servo_target_positions[list_index] = positions_list[current_index];
                                }
                            } else {
                                if ((object_keys.get(i + 1) == 2) || (key_index < 20)) {
                                    servo_target_positions[list_index] += 0.01 * object_keys.get(i + 3) * delta_time * (1 - object_keys.get(i + 2)) + object_keys.get(i + 2) * Math.min(1, (current_time - servo_times_started[list_index]) / 0.75);
                                } else { //if default axis
                                    servo_target_positions[list_index] += 0.01 * delta_time * (
                                        (key_index > 23) ? object_keys.get(i + 2) :
                                        (axes[key_index - 20] < 0 ? object_keys.get(i + 2) : object_keys.get(i + 3))
                                    );
                                }
                                servo_target_positions[list_index] = Math.max(Math.min(servo_target_positions[list_index], servo_max_positions[list_index]), servo_min_positions[list_index]);
                                servo_list[list_index].setPosition(servo_target_positions[list_index]);
                            }
                        }
                    }
                }
            } else if (cr_servo_names.contains(element.getKey())) {
                list_index = cr_servo_names.indexOf(element.getKey());

                if (!object_is_active) {
                    cr_servo_list[list_index].setPower(0);

                    cr_times_started[list_index] = -10.0;
                } else {

                    if (cr_times_started[list_index] < 0) cr_times_started[list_index] = current_time;
                    for (int i = 0; i < number_of_keys; i += 4) {
                        key_index = object_keys.get(i);
                        if (((key_index < 20) && (buttons[key_index])) || ((key_index > 19) && (axes[key_index - 20] > 0.1))) {
                            if ((object_keys.get(i + 1) == 2) || (key_index < 20)) {
                                cr_servo_list[list_index].setPower(0.01 * object_keys.get(i + 3) * (1 - object_keys.get(i + 2)) + object_keys.get(i + 2) * Math.min(1, (current_time - cr_times_started[list_index]) / 0.75));
                            } else {
                                cr_servo_list[list_index].setPower(axes[key_index - 20] * 0.01 * (
                                    (key_index > 23) ? object_keys.get(i + 2) : //if it's a trigger, then set it to the first val
                                    (axes[key_index - 20] < 0 ? object_keys.get(i + 2) : object_keys.get(i + 3))
                                ));
                            }
                        }
                    }
                }
            } else {
                for (int i = 0; i < number_of_keys; i++) {

                    key_index = object_keys.get(i); //where button is in list of keys; < 20 -> button, >= 20 -> axis

                    if ((key_index < 20 && buttons[key_index]) || (key_index > 19 && Math.abs(axes[key_index - 20]) > 0.1)) {
                        target_x = 0.001 * object_keys.get(i + 1);
                        target_y = 0.001 * object_keys.get(i + 2);
                        if (object_keys.get(i + 3) != 1) {
                            target_angle = Math.toRadians(0.001 * object_keys.get(i + 3));
                        } else {
                            target_angle = current_angle;
                        }
                    }
                }
            }
        }
    }



    public void new_keybind(String motor, String button, Object modifier1, Object modifier2, Object modifier3) {
        if (!keybinds.containsKey(motor)) throw new IllegalArgumentException("You misspelled " + motor + " - make sure its exactly as it's spelled in dc motor list or servo list, or it's \"goto\". Idiot");
        if (!(keys.contains(button))) throw new IllegalArgumentException("You misspelled " + button + "  - make sure its exactly as it's spelled in keys. ");

        int button_index = keys.indexOf(button);
        int mod1;
        int mod2;
        int mod3;
        if (motor.equals("goto")) {
            mod1 = (int) ((double) modifier1 * 1000);
            mod2 = (int) ((double) modifier2 * 1000);
            try {
                mod3 = (int) ((double) modifier3 * 1000);
            } catch (ClassCastException e) {
                mod3 = 1;
            }
            if (button_types[button_index] == 0) {
                button_types[button_index] = 3; //1 = default, 2 = toggle, 3 = button
            } else if (button_types[button_index] != 3) {
                throw new IllegalArgumentException("A button cannot have 2 types; however, you are setting \"" + button +
                        "\" to be 2 different things. (\"goto\" is, by default, a button) ");
            }
        } else {
            mod1 = (new ArrayList<>(Arrays.asList("x", "default", "toggle", "button", "cycle"))).indexOf((String) modifier1);
            int temp = Math.min(mod1, 3);
            if (button_types[button_index] == 0) {
                button_types[button_index] = temp; //1 = default, 2 = toggle, 3 = button
            } else if (button_types[button_index] != temp) {
                throw new IllegalArgumentException("A button cannot have 2 types; however, you are setting \"" + button +
                        "\" to be 2 different things. (\"goto\" is, by default, a button) ");
            }
            if (cr_servo_names.contains(motor)) {
                if (mod1 > 2) {
                    throw new IllegalArgumentException("CR Servos cannot be toggles or buttons. ");
                }
                if ((button_index < 20) || (mod1 == 2)) {
                    mod2 = (new ArrayList<>(Arrays.asList("normal", "gradient"))).indexOf((String) modifier1);
                    if (mod2 < 0) throw new IllegalArgumentException ("You misspelled \"normal\" or \"gradient\" (Case matters)");
                } else {
                    mod2 = (int) ((double) modifier2 * 100);
                    if (Math.abs(mod2) > 100) throw new IllegalArgumentException("Power has to be between -1 and 1");
                }
                mod3 = (int) ((double) modifier2 * 100);
                if (Math.abs(mod3) > 100) throw new IllegalArgumentException("Power has to be between -1 and 1");
            } else { //dc motor or servo
                if (mod1 > 2) {
                    mod2 = (int) modifier2;
                    mod3 = (int) modifier3;
                    if (mod3 >= positions.length) throw new IllegalArgumentException("You have not put that list in the \"positions\" array. ");
                } else {
                    if ((button_index < 20) || (mod1 == 2)) {
                        mod2 = (new ArrayList<>(Arrays.asList("normal", "gradient"))).indexOf((String) modifier1);
                        if (mod2 < 0) throw new IllegalArgumentException ("You misspelled \"normal\" or \"gradient\" (Case matters)");
                    } else {
                        mod2 = (int) ((double) modifier2 * 100);
                        if (Math.abs(mod2) > 100) throw new IllegalArgumentException("Power has to be between -1 and 1");
                    }
                    mod3 = (int) ((double) modifier2 * 100);
                    if (Math.abs(mod3) > 100) throw new IllegalArgumentException("Power has to be between -1 and 1");
                }
            }
        }
        for (int i : new int[] {button_index, mod1, mod2, mod3}) {
            keybinds.get(motor).add(i);
        }
    }

    public void set_button_types() {
        for (int i = 0; i < 27; i++) button_types[i] = Math.max(button_types[i], 1);
    }

    public void resetZeroAngle() {
        if (useRoadRunner) {
            zero_angle = 0 - position_tracker.getPoseEstimate().getHeading();
        } else if (usePID) {
            zero_angle = 0 - getAngle();
        }
    }

    public void setZeroAngle(double angle) {
        zero_angle = 0 - Math.toRadians(angle);
    }

    public void initializeRoadRunner(double x, double y, double angle, StandardTrackingWheelLocalizer localizer) {
        position_tracker = localizer;
        position_tracker.setPoseEstimate(new Pose2d(x, y, Math.toRadians(angle)));
        current_x = x;
        current_y = y;
        current_angle = Math.toRadians(angle);
        target_x = x;
        target_y = y;
        target_angle = Math.toRadians(angle);
        zero_angle -= Math.toRadians(angle);
    }

    //RoadRunner

    public double angle() { return modifiedAngle(0 - zero_angle - current_angle); }

    public double[][] robot_hitbox() {
        return new double[][] {
                {current_x + robot_length * Math.cos(angle()) - robot_width * Math.sin(angle()), current_y + robot_length * Math.sin(angle()) + robot_width * Math.cos(angle())},
                {current_x + robot_length * Math.cos(angle()) + robot_width * Math.sin(angle()), current_y + robot_length * Math.sin(angle()) - robot_width * Math.cos(angle())},
                {current_x - robot_length * Math.cos(angle()) + robot_width * Math.sin(angle()), current_y - robot_length * Math.sin(angle()) - robot_width * Math.cos(angle())},
                {current_x - robot_length * Math.cos(angle()) - robot_width * Math.sin(angle()), current_y - robot_length * Math.sin(angle()) + robot_width * Math.cos(angle())}
        };
    }

    public double distance_from(double[] point) {
        return Math.sqrt((point[0] - current_x) * (point[0] - current_x) + (point[1] - current_y) * (point[1] - current_y));
    }

    public boolean point_above_line(double[] point, double[][] line) { //definition of "above": y above line, or if vertical, then x value greater
        //point format:{x, y}
        //line format: {{x1, y1}, {x2, y2}}
        if (line[0][0] == line[1][0]) {
            return point[0] > line[0][0];
        } else if (point[0] == line[0][0]) {
            return point[1] > line[0][1];
        } else if (point[0] > line[0][0]) {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) > (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        } else {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) < (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        }
    }

    public boolean point_on_line(double[] point, double[][] line) {
        if (Math.abs(line[1][0] - line[0][0]) == 0) {
            return (Math.abs(line[0][0] - point[0]) < 0.01);
        } else if (Math.abs(point[0] - line[0][0]) == 0) {
            return false;
        } else {
            return (point[1] - line[0][1]) / (point[0] - line[0][0]) == (line[1][1] - line[0][1]) / (line[1][0] - line[0][0]);
        }
    }

    public boolean intersect(double[][] line1, double[][] line2) {

        if ((Math.max(line1[0][0], line1[1][0]) < Math.min(line2[0][0], line2[1][0])) || (Math.min(line1[0][0], line1[1][0]) > Math.max(line2[0][0], line2[1][0])) ||
                (Math.max(line1[0][1], line1[1][1]) < Math.min(line2[0][1], line2[1][1])) || (Math.min(line1[0][1], line1[1][1]) > Math.max(line2[0][1], line2[1][1]))) {
            return false;
        }

        if (line1[0][0] == line1[1][0]) {
            line1[1][0] += 0.01;
        }
        if (line2[0][0] == line2[1][0]) {
            line2[1][0] += 0.01;
        }

        if (point_on_line(line1[0], line2) || point_on_line (line1[1], line2) || point_on_line(line2[0], line1) || point_on_line (line2[1], line1)) {
            return false;
        } else {
            return (((point_above_line(line1[0], line2) != point_above_line(line1[1], line2))) && ((point_above_line(line2[0], line1) != point_above_line(line2[1], line1))));
        }
    }

    public boolean point_inside_polygon(double[] point, double[][] polygon, int accuracy) { //we want the inside-ness to be the same for every line
        double min_x = 10000000;
        double min_y = 10000000;
        double max_x = -10000000;
        double max_y = -10000000;
        int intersections;
        double[][] radial_line = new double[2][2];
        double[][] next_line = new double[2][2];

        for (double[] i : polygon) {
            min_x = Math.min(min_x, i[0]);
            max_x = Math.max(max_x, i[0]);
            min_y = Math.min(min_y, i[1]);
            max_y = Math.max(max_y, i[1]);
        }
        double length = Math.sqrt((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y));

        for (int i = 0; i < accuracy; i++) {
            intersections = 0;
            radial_line[0][0] = point[0];
            radial_line[0][1] = point[1];
            radial_line[1][0] = point[0] + length * Math.cos(2.0 * Math.PI / (double) accuracy * (double) i);
            radial_line[1][1] = point[1] + length * Math.sin(2.0 * Math.PI / (double) accuracy * (double) i);

            for (int j = 0; j < polygon.length; j++) { //add 1 if intersects line, 0.5 for each endpoint it touches
                next_line[0] = polygon[j];
                next_line[1] = polygon[(j + 1) % polygon.length];
                if (point_on_line(polygon[j], radial_line)) {
                    intersections += 1;
                }
                if (point_on_line(polygon[(j + 1) % polygon.length], radial_line)) {
                    intersections += 1;
                }
                if (intersect(radial_line, next_line)) {
                    intersections += 2;
                }
            }
            if (intersections % 4 != 2) {
                return false;
            }
        }
        return true;
    }

    public boolean completely_inside(double[][] polygon1, double[][] polygon2, int accuracy) {
        for (double[] point : polygon1) {
            if (!point_inside_polygon (point, polygon2, accuracy))
                return false;
        }
        return true;
    }

    public boolean shells_intersect(double[][] polygon1, double[][] polygon2) { //only seeing if the outer shells interesect each other
        for (int i = 0; i < polygon1.length; i++) {
            for (int j = 0; j < polygon2.length; j++) {
                if (intersect(new double[][] {polygon1[i], polygon1[(i + 1) % polygon1.length]}, new double[][] {polygon2[j], polygon2[(j + 1) % polygon2.length]}))
                    return true;
            }
        }
        return false;
    }

    public boolean polygons_intersect(double[][] polygon1, double[][] polygon2, int accuracy) {
        return (shells_intersect(polygon1, polygon2) || (completely_inside(polygon1, polygon2, accuracy) || completely_inside(polygon2, polygon1, accuracy)));
    }

    public boolean inside_polygon(double[][] polygon) {
        return completely_inside(robot_hitbox(), polygon, 6);
    }

    public boolean robot_on_point(double[] point) {
        return point_inside_polygon(point, robot_hitbox(), 6);
    }

    public boolean facing_polygon(double[][] polygon, double max_length, double offset) {
        double[][] radial_line = {{current_x, current_y}, {current_x + max_length * Math.cos(angle() + Math.toRadians(offset)), current_y + max_length * Math.sin(angle() + Math.toRadians(offset))}};
        double[][] next_line = new double[2][2];

        for (int j = 0; j < polygon.length; j++) { //add 1 if intersects line, 0.5 for each endpoint it touches
            next_line[0] = polygon[j];
            next_line[1] = polygon[(j + 1) % polygon.length];
            if (intersect(radial_line, next_line)) {
                return true;
            }
        }
        return false;
    }
}

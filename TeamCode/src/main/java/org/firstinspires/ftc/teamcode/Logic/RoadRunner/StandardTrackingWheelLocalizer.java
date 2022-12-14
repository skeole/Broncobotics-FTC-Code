package org.firstinspires.ftc.teamcode.Logic.RoadRunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robots;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public double TICKS_PER_REV;
    public double WHEEL_RADIUS; // in
    public double GEAR_RATIO; // output (wheel) speed / input (encoder) speed
    public boolean integer_overflow;
    public double forward_multiplier;
    public double strafing_multiplier;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, Robots r) {
        super(Arrays.asList(
                new Pose2d(0, r.lateral_distance * r.turning_multiplier / 2, 0), // left
                new Pose2d(0, 0 - r.lateral_distance * r.turning_multiplier / 2, 0), // right
                new Pose2d(r.forward_offset, 0, Math.toRadians(90)) // front
        ));
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, r.encoderNames.get(0)));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, r.encoderNames.get(1)));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, r.encoderNames.get(2)));

        TICKS_PER_REV = r.ticks_per_revolution;
        WHEEL_RADIUS = r.wheel_radius;
        GEAR_RATIO = r.gear_ratio;

        integer_overflow = r.integer_overflow;

        forward_multiplier = r.forward_multiplier;
        strafing_multiplier = r.strafing_multiplier;

        for (int i = 0; i < 3; i++) {
            if (r.invert_encoders[i]) {
                switch (i) {
                    case 0:
                        leftEncoder.setDirection(Encoder.Direction.REVERSE);
                    case 1:
                        rightEncoder.setDirection(Encoder.Direction.REVERSE);
                    case 2:
                        frontEncoder.setDirection(Encoder.Direction.REVERSE);
                }
            }
        }
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * forward_multiplier,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * forward_multiplier,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * strafing_multiplier
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        if (integer_overflow) {
            return Arrays.asList(
                    encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * forward_multiplier,
                    encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * forward_multiplier,
                    encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * strafing_multiplier
            );
        }

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * forward_multiplier,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * forward_multiplier,
                encoderTicksToInches(frontEncoder.getRawVelocity()) * strafing_multiplier
        );
    }
}
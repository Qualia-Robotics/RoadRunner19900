package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private DcMotorEx leftintake,rightintake;

    private static final double GATE_OPEN_POS = 0.4;
    private static final double GATE_CLOSED_POS = 0.1;

    // ✅ Constructor
    public Intake(HardwareMap hardwareMap) {
        rightintake = hardwareMap.get(DcMotorEx.class, "rightintake");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftintake");
    }

    // ✅ Shoot action

    public class IntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftintake.setPower(0.8);
            rightintake.setPower(-0.8); // likely correct for mirrored motors
            return false;
        }
    }
    public Action Intaking() {
        return new IntakingAction();
    }

    // ✅ Stop shooting action


    private class StopIntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftintake.setPower(0);
            rightintake.setPower(0);
            return false;
        }
        public Action stopIntake() {

            return new StopIntakingAction();
        }
    }
}
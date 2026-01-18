package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Intake {

    private DcMotorEx leftintake,rightintake;
    private CRServo kickerServo;

    // ✅ Constructor
    public Intake(HardwareMap hardwareMap) {
        rightintake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        leftintake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        kickerServo = hardwareMap.get(CRServo.class, "kickerServo");
    }

    // ✅ Shoot action
    public Action Intaking() {

        return new IntakingAction();
    }
    public class IntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftintake.setPower(-0.6);
            rightintake.setPower(0.6); // likely correct for mirrored motors
            kickerServo.setPower(1.0);
            return false;
        }
    }

    public Action FastIntaking() {

        return new FastIntakingAction();
    }
    public class FastIntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftintake.setPower(-1);
            rightintake.setPower(1); // likely correct for mirrored motors
            kickerServo.setPower(1.0);
            return false;
        }
    }

    // ✅ Stop shooting action

    public Action stopIntake() {

        return new StopIntakingAction();
    }
    public class StopIntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftintake.setPower(0);
            rightintake.setPower(0);
            kickerServo.setPower(0);
            return false;
        }
    }
}
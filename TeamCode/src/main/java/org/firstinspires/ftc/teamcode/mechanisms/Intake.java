package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class Intake {

    private DcMotorEx leftIntake,rightIntake;
    private CRServo kickerServo;



    // ✅ Constructor
    public Intake(HardwareMap hardwareMap) {
        rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
        leftIntake = hardwareMap.get(DcMotorEx.class, "leftIntake");
        kickerServo = hardwareMap.get(CRServo.class, "kickerServo");
        leftIntake.setDirection(DcMotor.Direction.REVERSE);


    }



    // ✅ Shoot action
    public Action Intaking() {

        return new IntakingAction();
    }
    public class IntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftIntake.setPower(0.6);
            rightIntake.setPower(0.6); // likely correct for mirrored motors
            return false;
        }
    }

    public Action FastIntaking() {

        return new FastIntakingAction();
    }
    public class FastIntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftIntake.setPower(1);
            rightIntake.setPower(1); // likely correct for mirrored motors
            return false;
        }
    }
    public Action ReverseIntaking() {

        return new ReverseIntakingAction();
    }
    public class ReverseIntakingAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftIntake.setPower(-0.2);
            rightIntake.setPower(-0.2); // likely correct for mirrored motors
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
            leftIntake.setPower(0);
            rightIntake.setPower(0);
            return false;
        }
    }
}
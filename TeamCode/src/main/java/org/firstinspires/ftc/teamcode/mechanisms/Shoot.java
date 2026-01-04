package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shoot {

    private DcMotorEx leftShootMotor, rightShootMotor;
    private Servo gateServo;

    private static final double GATE_OPEN_POS = 0.5;
    private static final double GATE_CLOSED_POS = 0.1;

    // ✅ Constructor
    public Shoot(HardwareMap hardwareMap) {
        leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
    }

    // ✅ Shoot action
    public Action shootScore() {
        return new ShootToScore();
    }
    public class ShootToScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftShootMotor.setPower(0.6);
            rightShootMotor.setPower(-0.6); // likely correct for mirrored motors
            gateServo.setPosition(GATE_OPEN_POS);
            return false;
        }
    }

    // ✅ Stop shooting action

    public Action stopShooting() {
        return new NoShooting();
    }
    public class NoShooting implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftShootMotor.setPower(0.0);
            rightShootMotor.setPower(0.0);
            gateServo.setPosition(GATE_CLOSED_POS);
            return false;
        }

    }
}

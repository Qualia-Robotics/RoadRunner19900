package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate {

    private Servo rightGateServo, leftGateServo;
    private final double RIGHT_GATE_OPEN_POS = 0.2167;
    private final double LEFT_GATE_OPEN_POS = 0.2167;
    private final double RIGHT_GATE_CLOSED_POS = 0.095;
    private final double LEFT_GATE_CLOSED_POS = 0.095;



    // ✅ Constructor
    public Gate(HardwareMap hardwareMap) {
        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");


    }



    // ✅ Shoot action
    public Action Open() {

        return new GateOpen();
    }
    public class GateOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
            leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
            return false;
        }
    }
    public Action Close () {

        return new GateClose();
    }
    public class GateClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
            leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
            return false;
        }
    }


    }

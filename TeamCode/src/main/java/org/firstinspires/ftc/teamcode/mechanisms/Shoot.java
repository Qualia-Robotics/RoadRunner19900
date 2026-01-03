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
    private final double GATE_OPEN_POS = .4;    // adjust
    private final double GATE_CLOSED_POS = 0.1;  // adjust
    
    public Object Shoot(HardwareMap hardwareMap) {
        DcMotorEx leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        DcMotorEx rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        

             class shoot_To_Score implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    leftShootMotor.setPower(0.8);
                    rightShootMotor.setPower(0.8);
                    gateServo.setPosition(0.4);
                    return false;
                }
            }
             Action shoot_score() {
                return new shoot_To_Score();
            }

            public class NoShooting implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    gateServo.setPosition(-4);
                    return false;
                }
            }
            Action stopShooting() {
                return new NoShooting();
            }
        }
    }
    


    


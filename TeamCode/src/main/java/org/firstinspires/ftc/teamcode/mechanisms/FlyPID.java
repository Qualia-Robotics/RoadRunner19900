package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class FlyPID {

    private final DcMotorEx leftShootMotor;
    private final DcMotorEx rightShootMotor;
    private final Servo gateServo, leftGateServo;

    public static final double TARGET_VELOCITY = 1250;
    public static final double IDLE_VELOCITY = TARGET_VELOCITY / 2.0; // 575
    private static final double GATE_OPEN_POS = 0;
    private static final double GATE_CLOSED_POS = 0.5;
    private static final double LEFT_GATE_CLOSED_POS = 0;
    private static final double LEFT_GATE_OPEN_POS = 0.5;

    public void manualPower(double power) {
        leftShootMotor.setPower(power);
        rightShootMotor.setPower(power);
    }

    public double getVelocity() {
        return leftShootMotor.getVelocity();
    }

    public FlyPID(HardwareMap hardwareMap) {
        leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");

        leftShootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //p Adjusts how fast we get to speed
        //i
        //d Limits change in velocity
        //f provides an anticipatory, open-loop control input that helps motors quickly reach a target speed or position by directly countering known forces like gravity or friction, reducing reliance on the feedback loop to correct errors and making the system more responsive and stable, especially for velocity control
        PIDFCoefficients shooterPID = new PIDFCoefficients(
                55.0,
                0.0,
                0.0,
                15);
        leftShootMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                shooterPID
        );
    }

    /** Call every loop while active */
    public Action spinUp() {
        return packet -> {
            gateServo.setPosition(GATE_OPEN_POS);
            leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
            leftShootMotor.setVelocity(TARGET_VELOCITY);
            rightShootMotor.setPower(1.0);
            double velocity = leftShootMotor.getVelocity();
            packet.put("Flywheel TPS", velocity);
            packet.put("At Speed", velocity >= TARGET_VELOCITY * 0.97);

            return false; // keep running
        };
    }

    /** Call every loop while idle */
    public Action idle() {
        return packet -> {
            gateServo.setPosition(GATE_CLOSED_POS);
            leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
            leftShootMotor.setVelocity(IDLE_VELOCITY);
            rightShootMotor.setPower(0.5); // mirrors half-speed behavior

            double velocity = leftShootMotor.getVelocity();
            packet.put("Flywheel TPS", velocity);
            packet.put("Flywheel Mode", "IDLE");

            return false; // keep running
        };
    }


    public Action stop() {
        return packet -> {
            gateServo.setPosition(GATE_CLOSED_POS);
            leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
            return true; // finishes immediately
        };
    }
    public boolean atIdleSpeed() {
        return leftShootMotor.getVelocity() >= IDLE_VELOCITY * 0.97;
    }
    public boolean atSpeed() {
        return leftShootMotor.getVelocity() >= TARGET_VELOCITY * 0.97;
    }
}


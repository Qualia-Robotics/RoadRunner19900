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
    private final Servo rightGateServo, leftGateServo;

    public static final double RANGE_10 = 1000;

    public double TARGET_VELOCITY = 995;
    public static final double MID_VELOCITY = 1400;
    public static final double FAR_VELOCITY = 1340.67;


    public double calculateRPM(double distance) {
    // cubic regression based on 10 pts
    return 0.016835*(distance)*(distance)*(distance)-3.01515*(distance)*(distance)+180.867*(distance)-2580.80808;
    }
    //0.05*(distance)*(distance)-0.6*(distance)+1050.67;

    public static final double IDLE_VELOCITY = RANGE_10 / 1.1; // 575
    private final double RIGHT_GATE_OPEN_POS = 0.2167;
    private final double LEFT_GATE_OPEN_POS = 0.2167;
    private final double RIGHT_GATE_CLOSED_POS = 0.095;
    private final double LEFT_GATE_CLOSED_POS = 0.095;

    //double f = 0.0006;

   /* public double BangBangChicken
            (double cur, double vel) {
        if (cur < vel) {
            return 1;
        } else {
            return f * vel;
        }
    }*/

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
        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");

        leftShootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        //p Adjusts how fast we get to speed
        //i
        //d Limits change in velocity
        //f provides an anticipatory, open-loop control input that helps motors quickly reach a target speed or position by directly countering known forces like gravity or friction, reducing reliance on the feedback loop to correct errors and making the system more responsive and stable, especially for velocity control
        PIDFCoefficients shooterPID = new PIDFCoefficients(
                90.0,
                0.0,
                0.0,
                12);
        leftShootMotor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                shooterPID
        );
    }

    /** Call every loop while active */

    public Action spinUp() {
        return packet -> {
            rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
            leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
            leftShootMotor.setVelocity(TARGET_VELOCITY);
            rightShootMotor.setPower(1.0);
            double velocity = leftShootMotor.getVelocity();
            packet.put("Flywheel TPS", velocity);
            packet.put("At Speed", velocity >= TARGET_VELOCITY * 0.97);

            return false; // keep running
        };
    }
    public Action spinUpFar() {
        return packet -> {

            leftShootMotor.setVelocity(FAR_VELOCITY);
            rightShootMotor.setPower(1.0);
            double velocity = leftShootMotor.getVelocity();
            packet.put("Far Flywheel TPS", velocity);
            packet.put("At Far Speed", velocity >= FAR_VELOCITY * 0.97);

            rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
            leftGateServo.setPosition(LEFT_GATE_OPEN_POS);

            return false; // keep running
        };
    }
    /** Call every loop while idle */
    public Action idle() {
        return packet -> {
            rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
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
            rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
            leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
            leftShootMotor.setPower(0);
            rightShootMotor.setPower(0);
            return false; // finishes immediately
        };
    }

    public Action spinUpCalc(double distance) {
        return packet -> {

            double targetRPM = calculateRPM(distance);
            double velocity = leftShootMotor.getVelocity();

            rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
            leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

            leftShootMotor.setVelocity(targetRPM);
            rightShootMotor.setPower(1.0);

            packet.put("Target RPM", targetRPM);
            packet.put("Current RPM", velocity);
            packet.put("At Speed", velocity >= targetRPM * 0.97);

            return false;
        };
    }


    public boolean atSpeed() { return leftShootMotor.getVelocity() >= TARGET_VELOCITY * 0.97; }
    public boolean atFarSpeed() {return leftShootMotor.getVelocity() >= FAR_VELOCITY * 0.97;}
    public boolean atMidSpeed() { return leftShootMotor.getVelocity() >= MID_VELOCITY * 0.97;}
    public boolean atCalcSpeed(double distance) {
        double targetRPM = calculateRPM(distance);
        return leftShootMotor.getVelocity() >= targetRPM * 0.94;  // instead of 0.97
    }


}


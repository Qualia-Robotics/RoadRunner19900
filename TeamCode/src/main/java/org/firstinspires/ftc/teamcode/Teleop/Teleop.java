package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.mechanisms.FlyPID;


import java.util.Locale;

@TeleOp(name="Teleop", group="1) Main OpModes")
public class Teleop extends LinearOpMode {

    // Gate positions
    private final double GATE_OPEN_POS = 0.1;
    private final double GATE_CLOSED_POS = 0.7;
    private final double LEFT_GATE_CLOSED_POS = 0.15;
    private final double LEFT_GATE_OPEN_POS = 0.75;

    // Drive motors
    private DcMotor fl, bl, fr, br;

    // Intake & shooter
    private DcMotor leftIntake, rightIntake;

    // Servos
    private CRServo kickerServo;
    private Servo gateServo, leftGateServo;

    // Constants
    private final double MAX_POWER = 0.8;

    // Drive variables
    private double forward, turn, strafe;

    GoBildaPinpointDriver pinpoint;
    double oldTime = 0;

    /* ================= SHOOT MACRO ================= */

    enum ShootState {
        IDLE,
        SPINUP,
        SPINUPFAR,
        SHOOT,
        DONE
    }

    ShootState shootState = ShootState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean lastCircle = false;

    /* =============================================== */

    @Override
    public void runOpMode() {

        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        kickerServo = hardwareMap.get(CRServo.class, "kickerServo");

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        FlyPID flywheel = new FlyPID(hardwareMap);
        Action flywheelAction = null;

        pinpoint.setOffsets(-84.0, 171.4, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        leftIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        gateServo.setPosition(GATE_CLOSED_POS);
        leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

        waitForStart();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            /* -------- DRIVE -------- */
            double precisionFactor = gamepad1.a ? 0.3 : 1.0;

            forward = -gamepad1.left_stick_y * MAX_POWER * precisionFactor;
            strafe  =  gamepad1.left_stick_x * MAX_POWER * precisionFactor;
            turn    =  gamepad1.right_stick_x * MAX_POWER * precisionFactor;

            fl.setPower(forward + strafe + turn);
            bl.setPower(forward - strafe + turn);
            fr.setPower(forward - strafe - turn);
            br.setPower(forward + strafe - turn);

            /* -------- INTAKE -------- */
            double intakePower = gamepad1.right_trigger * MAX_POWER;
            double outtakePower = gamepad1.left_trigger * MAX_POWER;
            double finalPower = intakePower - outtakePower;

            leftIntake.setPower(finalPower);
            rightIntake.setPower(finalPower);


            /* -------- SHOOTER MANUAL -------- */
            if (shootState == ShootState.IDLE) {
                if (gamepad1.dpad_up) {
                    flywheel.manualPower(0.5);
                    gateServo.setPosition(GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
                } else if (gamepad1.dpad_left) {
                    flywheel.manualPower(0.40);
                    gateServo.setPosition(GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
                } else if (gamepad1.dpad_down) {
                    gateServo.setPosition(GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
                    flywheel.stop().run(packet);
                }
            }

            if (gamepad1.x) {
                kickerServo.setPower(1.0);
                leftIntake.setPower(1.0);
                rightIntake.setPower(1.0);
            } else {
                kickerServo.setPower(0.0);
            }

            /* -------- SHOOT MACRO (CIRCLE) -------- */
            boolean circle = gamepad1.circle;
            if (circle && !lastCircle && shootState == ShootState.IDLE) {
                shootState = ShootState.SPINUP;
                shootTimer.reset();
            }
            lastCircle = circle;
            switch (shootState) {

                case SPINUP:
                    if (flywheelAction == null) {
                        flywheelAction = flywheel.spinUp();
                    }
                    flywheelAction.run(packet);


                    // gently pull balls down
                    if (shootTimer.seconds() < 0.2) {
                        leftIntake.setPower(-0.2);
                        rightIntake.setPower(-0.2);
                        kickerServo.setPower(-0.7);
                    } else {
                        leftIntake.setPower(0);
                        rightIntake.setPower(0);
                        kickerServo.setPower(-0.7);
                    }

                    // open gates
                    gateServo.setPosition(GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);

                    // advance once flywheel reaches speed
                    if (flywheel.atSpeed()) {
                        shootTimer.reset();
                        shootState = ShootState.SHOOT;
                    }

                    break;

                case SHOOT:
                    if (shootTimer.seconds() > 0.1) { // small buffer
                        leftIntake.setPower(1.0);
                        rightIntake.setPower(1.0);
                        kickerServo.setPower(1.0);
                    }

                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    flywheel.stop().run(packet);
                    flywheelAction = null;
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    kickerServo.setPower(0);
                    gateServo.setPosition(GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

                    shootState = ShootState.IDLE;
                    break;

            }

            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !lastCircle && shootState == ShootState.IDLE) {
                shootState = ShootState.SPINUPFAR;
                shootTimer.reset();
            }
            lastCircle = dpadUp; // keep the rising-edge logic the same

            switch (shootState) {

                case SPINUPFAR:
                    if (flywheelAction == null) {
                        flywheelAction = flywheel.spinUpFar();
                    }
                    flywheelAction.run(packet);

                    // gently pull balls down
                    if (shootTimer.seconds() < 0.2) {
                        leftIntake.setPower(-0.2);
                        rightIntake.setPower(-0.2);
                        kickerServo.setPower(-0.7);
                    } else {
                        leftIntake.setPower(0);
                        rightIntake.setPower(0);
                        kickerServo.setPower(-0.7);
                    }

                    // open gates
                    gateServo.setPosition(GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);

                    // advance once flywheel reaches speed
                    if (flywheel.atFarSpeed()) {
                        shootTimer.reset();
                        shootState = ShootState.SHOOT;
                    }

                    break;

                case SHOOT:
                    if (shootTimer.seconds() > 0.1) { // small buffer
                        leftIntake.setPower(1.0);
                        rightIntake.setPower(1.0);
                        kickerServo.setPower(1.0);
                    }

                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    flywheel.stop().run(packet);
                    flywheelAction = null;
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    kickerServo.setPower(0);
                    gateServo.setPosition(GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

                    shootState = ShootState.IDLE;
                    break;

            }


            /* -------- TELEMETRY -------- */
            Pose2D pos = pinpoint.getPosition();
            telemetry.addData("Position",
                    String.format(Locale.US, "{X: %.2f, Y: %.2f, H: %.2f}",
                            pos.getX(DistanceUnit.INCH),
                            pos.getY(DistanceUnit.INCH),
                            pos.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Flywheel TPS", flywheel.getVelocity());
            telemetry.addData("At speed?", flywheel.atSpeed());
            telemetry.addData("At Far speed?", flywheel.atFarSpeed());
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.addData("Far Flywheel TPS", flywheel.getVelocity());
            telemetry.update();
        }
    }
}

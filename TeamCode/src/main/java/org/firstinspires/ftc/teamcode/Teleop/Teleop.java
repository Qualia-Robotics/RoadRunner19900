package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="Teleop", group="1) Main OpModes")
public class Teleop extends LinearOpMode {

    // Gate positions
    private final double GATE_OPEN_POS = -0.1;
    private final double GATE_CLOSED_POS = 0.5;
    private final double LEFT_GATE_CLOSED_POS = 0;
    private final double LEFT_GATE_OPEN_POS = 0.5;

    // Drive motors
    private DcMotor fl, bl, fr, br;

    // Intake & shooter
    private DcMotor leftIntake, rightIntake, leftShootMotor, rightShootMotor;

    // Servos
    private CRServo kickerServo;
    private Servo gateServo, leftGateServo;

    // Constants
    private final double MAX_POWER = 0.8;
    private final double COUNTS_PER_REV = 1024.0;

    // Drive variables
    private double forward, turn, strafe;
    private double lastAngle = 0;

    GoBildaPinpointDriver pinpoint;
    double oldTime = 0;

    /* ================= SHOOT MACRO ================= */

    enum ShootState {
        IDLE,
        SPINUP,
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

        leftShootMotor = hardwareMap.get(DcMotor.class, "leftShootMotor");
        rightShootMotor = hardwareMap.get(DcMotor.class, "rightShootMotor");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        kickerServo = hardwareMap.get(CRServo.class, "kickerServo");

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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
            if (gamepad1.dpad_up) {
                leftShootMotor.setPower(0.70);
                rightShootMotor.setPower(-0.70);
                gateServo.setPosition(GATE_OPEN_POS);
                leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
            }

            if (gamepad1.dpad_down || gamepad1.dpad_left) {
                gateServo.setPosition(GATE_CLOSED_POS);
                leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
                leftShootMotor.setPower(0);
                rightShootMotor.setPower(0);
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
                    // Spin up flywheels
                    leftShootMotor.setPower(0.69);
                    rightShootMotor.setPower(-0.69);

                    // FIRST 0.10s: gently pull balls down
                    if (shootTimer.seconds() < 0.20) {
                        leftIntake.setPower(-0.20);
                        rightIntake.setPower(-0.20);
                        kickerServo.setPower(-0.2);
                    } else {
                        leftIntake.setPower(0);
                        rightIntake.setPower(0);
                        kickerServo.setPower(0);
                    }
                    // Open gates
                    gateServo.setPosition(GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);

                    // After spinup, go shoot
                    if (shootTimer.seconds() > 1.75) {
                        shootState = ShootState.SHOOT;
                        shootTimer.reset();
                    }
                    break;

                case SHOOT:
                    // Feed all balls at once
                    kickerServo.setPower(1.0);
                    leftIntake.setPower(1.0);
                    rightIntake.setPower(1.0);

                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    // Stop everything
                    kickerServo.setPower(0.0);
                    leftIntake.setPower(0.0);
                    rightIntake.setPower(0.0);

                    leftShootMotor.setPower(0);
                    rightShootMotor.setPower(0);

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

            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();

            sleep(20);
        }
    }
}

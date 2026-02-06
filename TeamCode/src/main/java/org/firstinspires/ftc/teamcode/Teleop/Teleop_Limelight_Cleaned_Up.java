package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.FlyPID;

import java.util.List;

@TeleOp(name="Teleop_Limelight_Cleaned_Up", group="1) Main OpModes")
public class Teleop_Limelight_Cleaned_Up extends LinearOpMode {

    /*------------------------------------ GATE POSITIONS ---------------------------------------- */
    private final double RIGHT_GATE_OPEN_POS = 0.2167;
    private final double LEFT_GATE_OPEN_POS = 0.2167;
    private final double RIGHT_GATE_CLOSED_POS = 0.1;
    private final double LEFT_GATE_CLOSED_POS = 0.1;
    /* ---------------------------------- HARDWARE VARIABLES --------------------------------------*/
    // Limelight
    private Limelight3A limelight;
    // Drive motors
    private DcMotor fl, bl, fr, br;
    // Intake & shooter
    private DcMotor leftIntake, rightIntake;
    // Servos
    private Servo rightGateServo, leftGateServo;
    // Constants
    private final double MAX_POWER = 0.8;
    /* ----------------------------  LIMELIGHT BASED PD controller --------------------------------*/
    double kP = 0.02;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //offset goal
    double angleTolerance = .4;
    double kD = 0.001;
    double curTime = 0;
    double lastTime = 0;
    // Drive variables
    private double forward, turn, strafe;
    /*------------------------- controller based PID tuning (temporary)----------------------------*/
    double[] stepSizes = {1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 2;
    /* ---------------------------------- SHOOT MACRO STATES ------------------------------------- */
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

    /* ------------------------------------------------------------------------------------------- */

    @Override
    public void runOpMode() {
    /* ----------------------------------- HARDWARE MAPPING ---------------------------------------*/
        fl = hardwareMap.get(DcMotor.class, "leftFront");
        bl = hardwareMap.get(DcMotor.class, "leftBack");
        fr = hardwareMap.get(DcMotor.class, "rightFront");
        br = hardwareMap.get(DcMotor.class, "rightBack");

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntake");

        rightGateServo = hardwareMap.get(Servo.class, "rightGateServo");
        leftGateServo = hardwareMap.get(Servo.class, "leftGateServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        FlyPID flywheel = new FlyPID(hardwareMap);
        Action flywheelAction = null;
        /* ---------------------------------- MOTOR DIRECTIONS ------------------------------------*/
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        /*-----------------------------------------------------------------------------------------*/
        leftIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
        leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        resetRuntime();
        curTime = getRuntime();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

        /* -------------------------------------- DRIVE ------------------------------------------ */
            forward = -gamepad1.left_stick_y;
            strafe  =  gamepad1.left_stick_x;
            turn    =  gamepad1.right_stick_x;

        /* -------------------------------- LIMELIGHT RESULTS -------------------------------------*/
            LLResult result = limelight.getLatestResult();
            LLResultTypes.FiducialResult tag24 = null;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == 24) {
                        tag24 = fr;
                        break;
                    }
                }
            }

            if (tag24 != null) {
                double tx = tag24.getTargetXDegrees();
                double ty = tag24.getTargetYDegrees();
                telemetry.addData("Tag24 tx", tx);
                telemetry.addData("Tag24 ty", ty);
            } else {
                telemetry.addData("Tag24", "not visible");
            }

        /*-------------------------------- AUTO ALIGN ROTATION LOGIC ------------------------------*/
            if (gamepad1.a) {
                if (tag24 != null) {
                    error = tag24.getTargetXDegrees() - goalX;

                    if (Math.abs(error) < angleTolerance) {
                        turn = 0;
                        lastError = 0; // new addition, test first
                    } else {
                        double pTerm = error * kP;

                        curTime = getRuntime();
                        double dT = curTime - lastTime;
                        double dTerm;
                        if (dT > 0.01) {
                            dTerm = ((error - lastError) / dT) * kD;
                        } else {
                            dTerm = 0;
                        }

                        turn = Range.clip(pTerm + dTerm, -0.4, 0.4);

                        lastError = error;
                        lastTime = curTime;
                    }
                } else {
                    turn = 0;
                    lastTime = getRuntime();
                    lastError = 0;
                }
            } else {
                lastError = 0;
                lastTime = getRuntime();

            }
            drive.drive(forward, strafe, turn);
        /* -------------------------------- ANALOG POWER INTAKE -----------------------------------*/
            double intakePower = gamepad1.right_trigger * MAX_POWER;
            double outtakePower = gamepad1.left_trigger * MAX_POWER;
            double finalPower = intakePower - outtakePower;

            leftIntake.setPower(finalPower);
            rightIntake.setPower(finalPower);

        /* ---------------------------------- MANUAL SHOOTING -------------------------------------*/
            if (shootState == ShootState.IDLE) {
                if (gamepad1.dpad_left) {
                    flywheel.manualPower(0.40);
                    rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);
                } else if (gamepad1.dpad_down) {
                    rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
                    flywheel.stop().run(packet);
                }
            }
        /*------------------------------ CLOSE SHOOT MACRO (CIRCLE) -------------------------------*/
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

                    // open gates
                    rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
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
                    }
                    //shoot state lasts for 1 second
                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    flywheel.stop().run(packet);
                    flywheelAction = null;
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

                    shootState = ShootState.IDLE;
                    break;

            }
        /*------------------------------ FAR SHOOT MACRO (DPAD UP) --------------------------------*/
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

                    // open gates
                    rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
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
                    }
                    //shoot state lasts for 1 second
                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    flywheel.stop().run(packet);
                    flywheelAction = null;
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);
                    shootState = ShootState.IDLE;
                    break;

            }

        /* ------------------------------------ TELEMETRY ---------------------------------------- */
            telemetry.addData("Flywheel TPS", flywheel.getVelocity());
            telemetry.addData("At speed?", flywheel.atSpeed());
            telemetry.addData("At Far speed?", flywheel.atFarSpeed());
            telemetry.addData("Far Flywheel TPS", flywheel.getVelocity());
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access barcode results
                List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                for (LLResultTypes.BarcodeResult br : barcodeResults) {
                    telemetry.addData("Barcode", "Data: %s", br.getData());
                }

                // Access classifier results
                List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                for (LLResultTypes.ClassifierResult cr : classifierResults) {
                    telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                }

                // Access detector results
                List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                for (LLResultTypes.DetectorResult dr : detectorResults) {
                    telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                }

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}

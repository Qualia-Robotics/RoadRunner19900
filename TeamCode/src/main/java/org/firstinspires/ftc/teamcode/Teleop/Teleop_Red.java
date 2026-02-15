package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
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

//@TeleOp(name="Teleop_Red", group="1) Main OpModes")
public class Teleop_Red extends LinearOpMode {

    /*------------------------------------ GATE POSITIONS ---------------------------------------- */
    private final double RIGHT_GATE_OPEN_POS = 0.2167;
    private final double LEFT_GATE_OPEN_POS = 0.2167;
    private final double RIGHT_GATE_CLOSED_POS = 0.095;
    private final double LEFT_GATE_CLOSED_POS = 0.095;
    /* ---------------------------------- HARDWARE VARIABLES --------------------------------------*/
    // Limelight
    private Limelight3A limelight;
    // Drive motors
    private DcMotor fl, bl, fr, br;
    // Intake & shooter
    private DcMotor leftIntake, rightIntake;
    // Servos
    private Servo rightGateServo, leftGateServo;
    private Servo light;
    // Constants
    private final double MAX_POWER = 0.8;
    /* ----------------------------  LIMELIGHT BASED PD controller --------------------------------*/
    double kP = 0.025;
    //kp 0.025 // kd 0.0024
    double error = 0;
    double lastError = 0;
    double goalX = -3; //offset goal
    double angleTolerance = .75;
    double kD = 0.0024;
    double curTime = 0;
    double lastTime = 0;
    // Drive variables
    private double forward, turn, strafe;
    double lastTurnCmd = 0;
    double maxTurnDelta = 0.12; // per loop
    private final double MAX_AUTO_TURN = 0.45;
    private final double MIN_AUTO_TURN = 0.09;
    private final double DRIVER_TURN_OVERRIDE = 0.15;
    double lockedShotDistance = 0;



    /* ---------------------------- LIMELIGHT DISTANCE THRESHOLDS ---------------------------- */
    // inches from goal
    private double lastSeenDist = 30;
    double targetDistance = 0;

    // FAR = anything above MID_DIST
    /*------------------------- controller based PID tuning (temporary)----------------------------*/
    double[] stepSizes = {10, 5 , 1, 0.5, 0.1, 0.05, 0.01 ,0.005, 0.001, 0.0005 , 0.0001};
    int stepIndex = 1;
    /* ---------------------------------- SHOOT MACRO STATES ------------------------------------- */
    enum ShootState {
        IDLE,
        SPINUP,
        SHOOT,
        DONE
    }
    ShootState shootState = ShootState.IDLE;
    ElapsedTime shootTimer = new ElapsedTime();
    boolean lastCircle = false;


    /* ------------------------------- -AUTO TURN LOGIC AND MATH --------------------------------- */
    private double calculateAutoTurn(double currentTxDegrees) {
        // Compute error and timing
        double error = currentTxDegrees - goalX;
        double curTime = getRuntime();
        double dt = Math.max(0.001, curTime - lastTime);

        // Deadband: within tolerance → no turn
        if (Math.abs(error) <= angleTolerance) {
            lastError = error;
            lastTurnCmd = 0;
            lastTime = curTime;
            return 0;
        }

        // PD controller
        double pTerm = kP * error;
        double dTerm = kD * (error - lastError) / dt;
        double output = pTerm + dTerm;

        // Enforce minimum turn power
        if (Math.abs(output) < MIN_AUTO_TURN) {
            output = Math.copySign(MIN_AUTO_TURN, output);
        }

        // Clip to max turn power
        output = Range.clip(output, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        // Smooth rate of change
        double delta = Range.clip(output - lastTurnCmd, -maxTurnDelta, maxTurnDelta);
        double smoothed = lastTurnCmd + delta;

        // Update state
        lastError = error;
        lastTurnCmd = smoothed;
        lastTime = curTime;

        return smoothed;
    }


    private void resetAutoAlignState() {
        error = 0;
        lastError = 0;
        lastTurnCmd = 0;
        lastTime = getRuntime();
    }

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

        light = hardwareMap.get(Servo.class, "light");

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

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        /*-----------------------------------------------------------------------------------------*/
        leftIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
        leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

        limelight.start();
        light.setPosition(0);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();



        resetRuntime();
        curTime = getRuntime();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            /* -------------------------------------- DRIVE ------------------------------------------ */
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            double driverTurn = gamepad1.right_stick_x;
            double autoTurn = 0.0;


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
            /* ------------------------------ LIMELIGHT DISTANCE MATH -------------------------------------*/
            double distanceFromLimelightToGoalInches = Double.NaN;

            if (tag24 != null) {
                double ty = tag24.getTargetYDegrees();

                double limelightMountAngleDegrees = 13.75;
                double limelightLensHeightInches = 11.1;
                double goalHeightInches = 29.5;

                double angleToGoalDegrees = limelightMountAngleDegrees + ty;
                double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

                distanceFromLimelightToGoalInches =
                        (goalHeightInches - limelightLensHeightInches)
                                / Math.tan(angleToGoalRadians);
            }
            /*-------------------------------- AUTO ALIGN ROTATION LOGIC ------------------------------*/

            if (gamepad1.a && tag24 != null) {

                autoTurn = calculateAutoTurn(tag24.getTargetXDegrees());
            } else {
                resetAutoAlignState();
            }

            if (tag24 != null) {
                    light.setPosition(0.6); // target is within range, turn on light
                } else {
                    light.setPosition(0);   // target outside range, turn off
                }


            boolean driverOverridingTurn = Math.abs(driverTurn) > DRIVER_TURN_OVERRIDE;
            if (driverOverridingTurn) {
                turn = Range.clip(driverTurn, -1.0, 1.0);
            } else {
                turn = Range.clip(driverTurn + autoTurn, -1.0, 1.0);
            }
            drive.drive(forward, strafe, turn);
        /* -------------------------------- ANALOG POWER INTAKE -----------------------------------*/
            double intakePower = gamepad1.right_trigger * MAX_POWER;
            double outtakePower = gamepad1.left_trigger * MAX_POWER;
            double finalPower = intakePower - outtakePower;

        /* ---------------------------------- MANUAL SHOOTING -------------------------------------*/
            if (shootState == ShootState.IDLE) {
                leftIntake.setPower(finalPower);
                rightIntake.setPower(finalPower);
            }
        /*------------------------------------ SHOOT MACRO ----------------------------------------*/

            boolean circle = gamepad1.circle;

            if (circle && !lastCircle && shootState == ShootState.IDLE) {

                if (!Double.isNaN(distanceFromLimelightToGoalInches)) {
                    lastSeenDist = Range.clip(distanceFromLimelightToGoalInches, 30, 130);
                }

                lockedShotDistance = lastSeenDist;  // LOCK IT HERE

                shootState = ShootState.SPINUP;
                shootTimer.reset();
            }


            lastCircle = circle;

            // Only update lastSeenDist when the tag is valid AND distance is real
            if (tag24 != null && !Double.isNaN(distanceFromLimelightToGoalInches)) {
                lastSeenDist = distanceFromLimelightToGoalInches;
            }

            switch (shootState) {

                case IDLE:
                    //nothing right now
                    break;
                case SPINUP:
                    flywheelAction = flywheel.spinUpCalc(lockedShotDistance);
                    flywheelAction.run(packet);

                    if (flywheel.atCalcSpeed(lockedShotDistance)) {
                        shootTimer.reset();
                        shootState = ShootState.SHOOT;
                    }
                    break;


                case SHOOT:
                    rightGateServo.setPosition(RIGHT_GATE_OPEN_POS);
                    leftGateServo.setPosition(LEFT_GATE_OPEN_POS);

                    if (shootTimer.seconds() > 0.1) {
                        leftIntake.setPower(1.0);
                        rightIntake.setPower(1.0);
                    }

                    if (shootTimer.seconds() > 1.0) {
                        shootState = ShootState.DONE;
                    }
                    break;

                case DONE:
                    if (flywheelAction != null) {
                        flywheel.stop().run(packet);
                        flywheelAction = null;
                    }

                    leftIntake.setPower(0);
                    rightIntake.setPower(0);

                    rightGateServo.setPosition(RIGHT_GATE_CLOSED_POS);
                    leftGateServo.setPosition(LEFT_GATE_CLOSED_POS);

                    shootState = ShootState.IDLE;
                    break;
            }

            /* -------------------------- STEP SIZE SWITCHER (TEMPORARY) ----------------------------*/
            if(gamepad1.dpadLeftWasPressed()){
                new SleepAction(1);
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if(gamepad1.leftBumperWasPressed()){
                new SleepAction(1);
                kP += stepSizes[stepIndex];
            }
            if(gamepad1.rightBumperWasPressed()){
                new SleepAction(1);
                kP -= stepSizes[stepIndex];
            }
            if(gamepad1.dpad_up){
                new SleepAction(1);
                kD += stepSizes[stepIndex];
            }
            if(gamepad1.dpad_down){
                new SleepAction(1);
                kD -= stepSizes[stepIndex];
            }
        /* ------------------------------------ TELEMETRY ---------------------------------------- */
            telemetry.addData("Flywheel TPS", flywheel.getVelocity());
            telemetry.addData("Target Distance", targetDistance);
            telemetry.addData("At Speed?", flywheel.atCalcSpeed(lastSeenDist));
            telemetry.addData("kP lbumper/rbumper", kP);
            telemetry.addData("kD dpad_up/dpad down", kD);
            telemetry.addData("Last Seen Distance:", lastSeenDist +"in");

            LLStatus status = limelight.getStatus();
            //telemetry.addData("Name", "%s",
                    //status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            if (result != null && result.isValid()) {
                // Access general information
                if (tag24 != null) {
                    double dist = distanceFromLimelightToGoalInches;
                    telemetry.addData("LL Distance (in", "%.2f", dist);
                } else {
                    telemetry.addData("LL Distance (in)", "Tag 24 not visible");
                }

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

package org.firstinspires.ftc.teamcode.mechanisms;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
public class FlyPID {

    private DcMotorEx leftShootMotor, rightShootMotor;

    private static final double GATE_OPEN_POS = -0.1;
    private static final double GATE_CLOSED_POS = 0.5;

    private final double LEFT_GATE_CLOSED_POS = 0;
    private final double LEFT_GATE_OPEN_POS = 0.5;
    double TARGET_VELOCITY = 2600; // ticks/sec


    // ✅ Constructor
    public class FlyWare {

        private DcMotorEx leftShootMotor;
        private DcMotorEx rightShootMotor;

        public FlyWare(HardwareMap hardwareMap) {
            leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
            leftShootMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftShootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
            rightShootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // STARTING PIDF VALUES — tune later
            PIDFCoefficients shooterPID = new PIDFCoefficients(20.0, 0.0, 2.0, 12.0);
            leftShootMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPID);
        }

        // ✅ Shoot action
        public Action FlyRunUpPID() {
            return new FlyRunUpPID();
        }

        public class FlyRunUpPID implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double TARGET_VELOCITY = 2600; // ticks/sec

                leftShootMotor.setVelocity(TARGET_VELOCITY);
                rightShootMotor.setPower(0.9); // follower motor slightly under 100%

                // Telemetry
                double velocity = leftShootMotor.getVelocity();
                double rpm = (velocity / 28.0) * 60.0;
                packet.put("Flywheel TPS", velocity);
                packet.put("Flywheel RPM", rpm);
                packet.put("At Speed", velocity > TARGET_VELOCITY * 0.97);

                return false; // keep running until you stop manually
            }
        }
    }


    public Action FlyRunAct() {
        return new FlyRun();
    }
    public class FlyRun implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftShootMotor.setVelocity(TARGET_VELOCITY);
            rightShootMotor.setPower(1.0); // follow motor
            return false;
        }
    }

    // ✅ Stop shooting action

    public Action FlyOffAct() {
        return new FlyOff();
    }
    public class FlyOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftShootMotor.setVelocity(TARGET_VELOCITY);
            rightShootMotor.setPower(1.0); // follow motor
            return false;
        }

    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class intake {
    private Telemetry telemetry;
    private DcMotor intake;

    public void IntakePower(double target) {
        intake.setPower(target);
    }
    public void update() {
        // Simple position return

    }

    public intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotor.class, "intake");


    }

    public Action setIntakeTargetRR(double target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intake.setPower(target);
                return false;
            }

        };

    }
    public Action updateIntakeRR() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                update();
                //Return true makes this update the entire autonomous period so u gotta manually stop it lol
                return true;
            }
        };
    }

}
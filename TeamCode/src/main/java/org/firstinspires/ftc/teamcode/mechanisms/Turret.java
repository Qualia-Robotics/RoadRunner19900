
package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;
public class Turret {
    private final Telemetry telemetry;
    public double turnang;
    private CRServo leftTurretServo, rightTurretServo;
    private DcMotor leftIntake;
    private final double COUNTS_PER_REV = 1024.0;


    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftIntake       = hardwareMap.get(DcMotor.class, "leftIntake");
        leftIntake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");

    }
    public void turrtang(){
        int ticks = leftIntake.getCurrentPosition();
        double angle = ((double) ticks / COUNTS_PER_REV) * 360.0 / 4 / 4;
        double lastAngle = angle;

    }
}

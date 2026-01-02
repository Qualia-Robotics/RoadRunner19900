package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shoot {

    private DcMotorEx leftShootMotor, rightShootMotor;

    public Shoot(HardwareMap hardwareMap) {
        DcMotorEx leftShootMotor = hardwareMap.get(DcMotorEx.class, "leftShootMotor");
        DcMotorEx rightShootMotor = hardwareMap.get(DcMotorEx.class, "rightShootMotor");
    }
    public class shoot_To_Score implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            double rightpow = 0;
            if (!initialized) {
                leftShootMotor.setPower(0.8);
                rightShootMotor.setPower(-0.8);
                rightpow = 0.8;
                initialized = true;
            }

            // checks lift's current position


            if (rightpow == 0.8) {
                // true causes the action to rerun
                return true;
            } else {
                // false stops action rerun
                leftShootMotor.setPower(0);
                rightShootMotor.setPower(0);
                return false;
            }
        }
    }
}

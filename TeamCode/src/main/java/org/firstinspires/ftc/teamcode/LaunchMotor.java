package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LaunchMotor extends OpMode {
    private GoBILDA5202Series LN;

    @Override
    public void init() {
        LN = hardwareMap.get(GoBILDA5202Series.class, "LN");
        ((DcMotorEx)LN).setMotorEnable();
        }

    @Override
    public void loop() {
        ((DcMotorEx) LN).setTargetPositionTolerance(10);
        ((DcMotorEx) LN).setVelocity(6);
    }
    }




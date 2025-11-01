package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.workBench1;
import org.firstinspires.ftc.teamcode.mechanisms.workBench;

@TeleOp
public class DCmotorPractice extends OpMode {
    workBench bench = new workBench();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        double motorSpeed = gamepad1.right_stick_y;
        bench.setMotorSpeed(motorSpeed);
        if (bench.isTouchSensorPressed()){
            bench.setMotorSpeed(0.5);
        }
        else{
            bench.setMotorSpeed(0.0);//stops the motor
        }
        telemetry.addData("Motor Revs", bench.getMotorRevs());
    }
}

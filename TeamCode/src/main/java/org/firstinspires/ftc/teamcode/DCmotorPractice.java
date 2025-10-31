package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.workBench;
import org.firstinspires.ftc.teamcode.mechanisms.valGamePad;

@TeleOp
public class DCmotorPractice extends OpMode {
    workBench bench = new workBench();

    @Override
    public void init(){
        bench.init(hardwareMap);
    }

    @Override
    public void loop(){
        bench.setMotorSpeed(0.5);
    }
}

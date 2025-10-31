package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This program will post "Status:Running" on the telemetry of the driver hub.
 * */
@TeleOp
public class frijoles extends OpMode{
    @Override
    public void init(){
        telemetry.addData("Status", "Running");
    }
    @Override
    public void loop(){

    }
}

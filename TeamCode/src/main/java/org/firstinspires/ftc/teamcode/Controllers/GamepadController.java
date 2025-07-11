package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadController {
    public static GamepadController instance;
    static boolean instanceSet = false;
    public static void setInstance(GamepadController gamepadController){
        instance = gamepadController;
        instanceSet = true;
    }
    public static GamepadController getInstance(){
        
        return instance;
    }
    public Gamepad gamepad1,gamepad2;
    public GamepadController(Gamepad gamepad1,Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
    }

}

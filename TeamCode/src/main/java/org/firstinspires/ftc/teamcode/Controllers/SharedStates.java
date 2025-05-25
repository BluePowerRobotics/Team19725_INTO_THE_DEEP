package org.firstinspires.ftc.teamcode.Controllers;

public class SharedStates {
    // 单例实例
    private static SharedStates instance;
    public static synchronized SharedStates getInstance() {
        if (instance == null) {
            instance = new SharedStates();
        }
        return instance;
    }



    public enum MODE {HIGH_CHAMBER,LOW_CHAMBER,HIGH_BASKET,LOW_BASKET}
    private MODE RUNMODE = MODE.HIGH_CHAMBER;
    public MODE getRUNMODE(){
        return  RUNMODE;
    }
    public void setRUNMODE(MODE RUNMODE){
        this.RUNMODE = RUNMODE;
    }



    private boolean AUTO = false;
    public boolean isAUTO(){
        return AUTO;
    }
    public void setAUTO(boolean AUTO){
        this.AUTO = AUTO;
    }



    private boolean CLIMBING = false;
    public boolean isCLIMBING(){
        return CLIMBING;
    }
    public void setCLIMBING(boolean CLIMBING){
        this.CLIMBING = CLIMBING;
    }
}

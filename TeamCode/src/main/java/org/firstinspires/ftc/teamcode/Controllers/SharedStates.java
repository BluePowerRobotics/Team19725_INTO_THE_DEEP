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



    public enum RUNMODE {
        HIGH_CHAMBER,LOW_CHAMBER,HIGH_BASKET,LOW_BASKET;
        // 缓存枚举数组以避免重复调用values()
        private static final RUNMODE[] VALUES = values();

        public RUNMODE next() {
            // 计算下一个位置（循环）
            return VALUES[(this.ordinal() + 1) % VALUES.length];
        }
    }
    private RUNMODE MODE = RUNMODE.HIGH_CHAMBER;
    public RUNMODE getMODE(){
        return MODE;
    }
    public void setMODE(RUNMODE MODE){
        this.MODE = MODE;
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

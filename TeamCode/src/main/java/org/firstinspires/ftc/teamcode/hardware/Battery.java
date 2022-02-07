package org.firstinspires.ftc.teamcode.hardware;

import org.openftc.revextensions2.ExpansionHubEx;

import java.util.Objects;

public class Battery {

    public static ExpansionHubEx expansionHub1;
    public static ExpansionHubEx expansionHub2;
    public static int BatteryCapacity = 3000;

    /**
     * Passes the hubs to the Battery Class
     * @param eh1
     *  Hub 1
     * @param eh2
     *  Hub 2, if present
     */
    public static synchronized void init(ExpansionHubEx eh1, ExpansionHubEx eh2) {
        expansionHub1 = eh1;
        expansionHub2 = eh2;
    }
    public static synchronized void init(ExpansionHubEx eh1) {
        expansionHub1 = eh1;
    }

    public static int percentage() {
        int result = 0;
        double v = Math.round(voltage()/1000);
        result = (int)Math.round((v-10)*25);
        if(result < 0)
            result = 0;
        return Math.round(result);
    }

    public static int timeRemaining(String timeUnit) {
        int p = percentage();
        double a = currentDraw();
        double ret;
        if ( a < 1 ) return -1;
        ret = Math.round(((float)p/100)*60*(BatteryCapacity/a));
        if (Objects.equals(timeUnit, "HOURS")) {
            ret = Math.round(ret/60);
        } else if (Objects.equals(timeUnit, "SECONDS")) {
            ret = Math.round(ret*60);
        }
        return (int)Math.round(ret);
    }

    public static float currentDraw() {
        if (expansionHub1 == null) throw new NullPointerException("Battery.expansionHub is Not Defined");
        float v1 = Math.round(expansionHub1.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));
        float v2 = (expansionHub2 == null) ?
                Math.round(expansionHub2.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS)) :
                Math.round(expansionHub1.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.MILLIAMPS));

        return (v1+v2)/2;
    }

    public static float voltage() {
        if (expansionHub1 == null) throw new NullPointerException("Battery.expansionHub is Not Defined");
        float v1 = Math.round(expansionHub1.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS));
        float v2 = (expansionHub2 == null) ?
                Math.round(expansionHub2.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS)) :
                Math.round(expansionHub1.read12vMonitor(ExpansionHubEx.VoltageUnits.MILLIVOLTS));
        return (v1+v2)/2;
    }


}

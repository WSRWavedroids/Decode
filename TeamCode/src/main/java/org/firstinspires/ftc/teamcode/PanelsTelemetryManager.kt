package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry;


public val panelsTelemetry = PanelsTelemetry.telemetry

public fun addData(key: String, data: Any) {
    panelsTelemetry.addData(key, data)
}

public fun updateTelemetry() {
    panelsTelemetry.update()
}

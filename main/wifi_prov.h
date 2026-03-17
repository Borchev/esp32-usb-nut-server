#pragma once

/* Start WiFi. If stored credentials exist in NVS, connects as STA.
 * Otherwise starts a SoftAP captive portal for provisioning.
 * Blocks until a STA connection is established. */
void wifi_prov_start(void);

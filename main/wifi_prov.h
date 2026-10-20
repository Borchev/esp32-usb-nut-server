#pragma once

/* Start WiFi. If stored credentials exist in NVS, connects as STA.
 * Otherwise, or if they fail, starts a SoftAP captive portal for
 * provisioning while still retrying any stored credentials.
 * Blocks until a STA connection is established. After that, if the
 * connection is lost it keeps retrying in the background until it
 * reconnects. */
void wifi_prov_start(void);

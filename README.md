Sump Discharge Monitor
----------------------

This is a simple ESP-8266 based monitor to track the discharge volume from a sump pump.
It requires a flow rate sensor something like a YF-DN40 (check eBay).

Data is sent to a local MQTT broker and then to a website where trending and stats 
and managed.

Although the pump's flow rate exceeds the sensor's max flow rating, the accuracy still
seems quite good.




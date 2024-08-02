# Alpha Comms

<pre>
Alpha Comms is an abstracted communication interface that can be configured to output messages to different devices/modems.


src/modem.cpp:              A node the wraps around Goby dynamic buffer for queuing messages and MAC manager for Time Division Multiple Access (TDMA).
config/goby.yaml:           The configuration file for the modem node. The user should select the device driver (evologics/seatrac) they would like to use and define their dynamic buffer.
config/evologics.yaml:      Specific configuration settings for the evologics modem/usbl.
config/seatrac.yaml:        Specific configuration settings for the seatrac modem/usbl.

src/alpha_acomms.cpp: A specific node that encodes Alpha's messages for acomms status, state, command and control.

Publishers:
~/modem/rx      All data received from the modem driver

Subscribers:
~/modem/tx      All data the user wants to send



Goby Instalation Instruction: 
https://github.com/GobySoft/goby/wiki/InstallingGoby
</pre>
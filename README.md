# MVP Acomms

<pre>
MVP Acomms is an abstracted acoustic communication interface that can be configured to output messages to different devices/modems.


src/modem.cpp:              A node the wraps around Goby dynamic buffer for queuing messages and MAC manager for Time Division Multiple Access (TDMA).
config/goby.yaml:           The configuration file for the modem node. The user should select the device driver (evologics/seatrac) they would like to use and define their dynamic buffer.
config/evologics.yaml:      Specific configuration settings for the evologics modem/usbl.
config/seatrac.yaml:        Specific configuration settings for the seatrac modem/usbl.

src/mvp_acomms.cpp:       A specific node that encodes MVP's messages for acomms status, state, command and control.

Publishers:
modem/rx                    All data received from the modem driver
usbl_track                  USBL tracking information

Subscribers:
modem/tx                    All data the user wants to send




Goby Instalation Instruction: 
https://github.com/GobySoft/goby/wiki/InstallingGoby
</pre>
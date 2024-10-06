# MVP Acomms
This repo contains utilites for acoustic modem data processing for mvp framework.
- Specific acoustic modem hardware drivers are stored in the `external` folder as submodles.
- `mvp_acomm_intefaces` is created just for compiling test.
- all modem customized msgs should go to the hardware driver repo.

# Nodes in mvp_acomm_utilities
### Acomm_geopoint_node
- Subscribed topics
    - A GeoPoseStamped Topic where the localization or GPS is referenced to
    - A UsblData Topic where raw USBL data is included

- Publish topics
    - GeoposeStamped of the USBL
    - GeoPointStamped of the modem
    - Publish a ENU frame (and a TF) attached to the reference GeoPoseStamped
    - Publish a TF between USBL and Modem.


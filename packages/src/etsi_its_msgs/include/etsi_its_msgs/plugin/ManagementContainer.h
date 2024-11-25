#define ETSI_ITS_MSGS_MESSAGE_MANAGEMENTCONTAINER_PLUGIN_CONSTRUCTOR \
    ManagementContainer_() : \
        detection_time(0), reference_time(0), \
        termination(TERMINATION_UNAVAILABLE), \
        validity_duration(VALIDITY_DURATION_DEFAULT), \
        transmission_interval(TRANSMISSION_INTERVAL_UNAVAILABLE) {}

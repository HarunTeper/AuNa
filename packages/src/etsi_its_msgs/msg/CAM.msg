std_msgs/Header header
ItsPduHeader its_header
uint16 generation_delta_time # milliseconds since 2004 modulo 2^16

# basic container
StationType station_type
ReferencePosition reference_position

# TODO either BasicVehicle or RSU
BasicVehicleContainerHighFrequency high_frequency_container

bool has_low_frequency_container
BasicVehicleContainerLowFrequency low_frequency_container

# TODO special vehicle container is missing

ActionID action_id
uint64 detection_time # ms since 2004
uint64 reference_time # ms since 2004
uint8 termination
ReferencePosition event_position
RelevanceDistance relevance_distance
RelevanceTrafficDirection relevance_traffic_direction
uint32 validity_duration # seconds
uint32 transmission_interval # milliseconds
StationType station_type

uint8 TERMINATION_CANCELLATION = 0
uint8 TERMINATION_NEGATION = 1
uint8 TERMINATION_UNAVAILABLE = 255

uint32 TRANSMISSION_INTERVAL_ONE_MILLISECOND = 1
uint32 TRANSMISSION_INTERVAL_TEN_SECONDS = 10000
uint32 TRANSMISSION_INTERVAL_UNAVAILABLE = 0

uint32 VALIDITY_DURATION_DEFAULT = 600
uint32 VALIDITY_DURATION_TIME_OF_DETECTION = 0
uint32 VALIDITY_DURATION_ONE_SECOND_AFTER_DETECTION = 1

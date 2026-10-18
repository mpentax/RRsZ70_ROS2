# Generated from docs/migration/interfaces.md; SI units.
std_msgs/Header header
z70_interfaces/EvidenceStamp evidence
unique_identifier_msgs/UUID track_id
string<=32 label
geometry_msgs/Point floor_contact
bool contact_valid
bool covariance_known
float64[9] contact_covariance
float32 width_m
bool width_valid
string<=128 calibration_id

# Generated from docs/migration/interfaces.md; SI units.
unique_identifier_msgs/UUID command_id
unique_identifier_msgs/UUID attempt_id
uint32 attempt_index
builtin_interfaces/Time requested_at
bool write_time_known
builtin_interfaces/Time write_at
builtin_interfaces/Time finished_at
uint8 dispatch
uint8 response
bool vendor_code_known
int32 vendor_code
string<=64 error_code
uint8 physical_completion
z70_interfaces/EvidenceStamp verification

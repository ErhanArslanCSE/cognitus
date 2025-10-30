# Anomaly detection event

std_msgs/Header header
string anomaly_type  # unusual_object, missing_object, unexpected_movement
string description
float32 confidence
string[] affected_objects

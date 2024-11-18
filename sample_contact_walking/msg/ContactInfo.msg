# Contact information for a single contact frame over a time horizon.

# Frame on the robot that made contact
string robot_contact_frame

# Swing times. Organized as [start_time1, end_time1, start_time2, end_time2, ...]
float64[] swing_times

# Contact polytopes associated with each contact phase
ContactPolytope[] polytopes


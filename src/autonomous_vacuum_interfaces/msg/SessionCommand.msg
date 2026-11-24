# Internal command issued by the task manager (or CLI during integration).
string session_id
string floor_id
bool running
string mode
string power_mode
# Available bin slots at the start of a cleaning task.
uint32 available_bin_cells 4294967295
# Set only after the docking action confirms arrival and standstill.
bool docked
# One task may have several working sessions separated by a docking stop.
string task_id
uint32 cleaned_cells_at_start

# One analysed frame of the dilute-to-volume flask camera. Rows grow downward.
builtin_interfaces/Time stamp
int32 x0                  # neck left edge column
int32 x1                  # neck right edge column
bool ring_found
float32 ring_row          # graduation ring row at the neck centre
float32 ring_strength     # grey levels
float32 ring_cover        # 0..1 fraction of the neck width
bool surface_found
float32 surf_top          # liquid surface band top row
float32 surf_bot          # band bottom row
float32 surf_low          # lower edge row at the neck centre (parabola fit)
float32 gap_px            # surf_low - ring_row (positive = liquid below the mark); 0 when either is missing

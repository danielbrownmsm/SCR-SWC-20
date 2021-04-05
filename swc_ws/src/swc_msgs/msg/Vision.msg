# for nearest waypoint
float64 x_offset # offset from crosshair
float32 distance # distance probably using lookup table or Limelight equation
bool detected # if nothing was found then don't bother looking at offset or dist

#Vector3[] obstacles # x, y, size
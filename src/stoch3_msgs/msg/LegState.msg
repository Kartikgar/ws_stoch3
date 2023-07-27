string name
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 force
float64 support_probability        # [0, 1] , probability that the leg is a support leg.
                                   # Limit the value to range [0, 1] if it is outside the range.

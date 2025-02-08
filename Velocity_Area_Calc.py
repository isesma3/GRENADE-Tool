def calculate_required_velocity(swath_width_km, mission_area_km2, mission_time_years):
    # velocities = []

    # Convert mission time from years to seconds
    mission_time_seconds = mission_time_years * 365.25 * 24 * 60 * 60  # Considering leap years with 365.25 days/year

    # Calculate the total length that needs to be covered by this sensor
    total_length_km = mission_area_km2 / swath_width_km

    # Calculate the required velocity in m/s to cover the total length within the mission time
    velocity_ms = total_length_km*1000 / mission_time_seconds

    # for sensor in sensor_specs:
    #     swath_width_km = sensor['swath_width']  # Swath width in kilometers

    #     # Calculate the total length that needs to be covered by this sensor
    #     total_length_km = mission_area_km2 / swath_width_km

    #     # Calculate the required velocity in kilometers per hour to cover the total length within the mission time
    #     velocity_ms = total_length_km*1000 / mission_time_hours

    #     velocities.append({
    #         'resolution': sensor['resolution'],
    #         'swath_width': swath_width_km,
    #         'required_velocity_kmph': velocity_ms,
    #         'total_length_km': total_length_km  # Adding total length to the results
    #     })

    return velocity_ms

def calculate_area_coverage(swath_width_km, velocity_ms, mission_time_years):
    
    # Convert mission time from years to seconds
    mission_time_seconds = mission_time_years * 365.25 * 24 * 60 * 60  # Considering leap years with 365.25 days/year
    
    # Calculate the total length covered in km
    total_length_km = (velocity_ms * mission_time_seconds)/1000
    
    # Calculate the area coverage
    mission_area_km2 = total_length_km * swath_width_km

    return mission_area_km2

def calculate_mission_t(swath_width_km, mission_area_km2, velocity_ms):

    # Calculate total length required
    total_length_km = mission_area_km2/swath_width_km

    # Calculate the mission time required in seconds
    mission_time_seconds = (total_length_km*1000)/velocity_ms

    # Calculate the mission time in years
    mission_time_years = mission_time_seconds/(365.25 * 24 * 60 * 60 )

    return mission_time_years

# # Example sensor specifications
# sensor_specs = [
#     {'resolution': 1, 'swath_width': 10},  # Resolution in meters, swath width in kilometers
#     {'resolution': 3, 'swath_width': 30},
#     {'resolution': 16, 'swath_width': 100}
# ]

# # Mission parameters
# mission_area_km2 = 4.5e6  # Mission area in square kilometers
# mission_time_years = 1  # Mission time in years

# # Calculate required velocities for each sensor
# velocities = calculate_required_velocity(10, mission_area_km2, mission_time_years)
# print(velocities)
# # Display the results
# # for v in velocities:
# #     print(f"Sensor with {v['resolution']}m resolution & {v['swath_width']}km swath width requires a velocity of {v['required_velocity_kmph']:.2f} m/s to cover Kraken Mare in {mission_time_years:.2f} years.")
# #     print(f"Total length to be covered: {v['total_length_km']:.2f} km")


# mission_area_km2 = calculate_area_coverage(sensor_specs[0], 14, 1)
# print(mission_area_km2)

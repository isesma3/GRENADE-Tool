#Aero Performance
#Based on longitudinal distance covered on Titan
from math import pi


def aero_perf(no_of_passes, velocity, circumference, mission_time, min_alt, max_alt):
    '''
    Calculating for 4 revololutions per iteration
    Multiple passes at Titan's poles since they are regions of interest
    '''
    #Calculating total dist to be covered
    alt_dist_btw_passes = (max_alt-min_alt)/no_of_passes

    radian_separation = pi/4
    total_dist_covered = (2*pi/radian_separation)*circumference*no_of_passes


    #time taken for single pass
    time_taken = total_dist_covered/velocity

    #time remaining for ground mapping
    time_remaining = mission_time*365.25*24*3600 - time_taken

    return [time_remaining, alt_dist_btw_passes]
'''

    while(power_required-power_actual<0):
'''

#Ground Performance
def grnd_perf(velocity,a,D):
    '''
    Should fly at 1.5km for proper output by LIDAR
    '''
    area_percentage_goal = 0.10
    area_total = 4*pi*(D/2)**2
    area_goal = area_percentage_goal*area_total
    altitude= 1500
    length_of_rect = 2251.7

    total_area_covered_single_instance = length_of_rect*velocity
    area_actual = total_area_covered_single_instance*a[0]

    G = area_actual / area_goal

    return G

#testing
#a = aero_perf(3, 10, 2*pi, 10, 100, 200)
#print(a)
#g = grnd_perf(10, a)
#print(g)
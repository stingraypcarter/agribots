import datetime

# Test locations must be obtained with RTK base configuration
hub_origin_ll = (42.274315,-71.8084567)
hub_origin_deg = 34.35103851903750 #deg

def ll2xy(ll,origin_ll,origin_deg,scalers):
    NE = ((ll[0]-origin_ll[0])*scalers[0],(ll[1]-origin_ll[1])*scalers[1])
    psi = deg2rad(-origin_deg)
    x = math.sin(psi)*NE[0] + math.cos(psi)*NE[1]
    y = math.cos(psi)*NE[0] - math.sin(psi)*NE[1]
    return (x,y)

def xy2ll(xy,origin_ll,origin_deg,scalers):
    psi = deg2rad(origin_deg)
    N = math.cos(psi)*xy[1]- math.sin(psi)*xy[1]
    E = math.sin(psi)*xy[1]+ math.cos(psi)*xy[1]
    return (origin_ll[0] + (N/scalers[0]),origin_ll[1] + (E/scalers[1]))

def deg2m(pos_gps):
    mu = pos_gps[0]*math.pi/180
    # constants from wikipedia https://en.wikipedia.org/wiki/Geographic_coordinate_system#Length_of_a_degree
    lat = 111132.92 - 559.82 * math.cos(2*mu) + 1.175 * math.cos(4*mu) -0.0023 * math.cos(6*mu)
    lon = 111412.84 * math.cos(mu) -93.5* math.cos(3*mu) + 0.118 * math.cos(5*mu)
    return (lat,lon)

def decode_GPS_RMC(str):
    data = str.split(',')
    header = data[0]
    str_UTC_time = data[1]
    str_time_valid = data[2]
    str_latitude = data[3]
    str_latitude_north = data[4]
    str_longitude = data[5]
    str_longitude_east = data[6]
    str_ground_speed = data[7]
    str_ground_course = data[8]
    str_UTC_date = data[9]

    # assuming UTC date is valid from error check bit
    str_datetime = str_UTC_time+'0000'+str_UTC_date
    datetime_obj = datetime.strptime(str_datetime,'%H%M%S.%f%d%m%y')

    latitude_north = str_latitude_north == 'N'
    longitude_east = str_longitude_east == 'E'

    latitude = float(str_latitude[0:2])+(float(str_latitude[2:])/60.0)
    if not latitude_north:
        latitude *= -1

    longitude = float(str_longitude[0:3])+(float(str_longitude[3:])/60.0)
    if not longitude_east:
        longitude *= -1
    GPS_pos = (latitude,longitude)

    return (datetime_obj,GPS_pos)

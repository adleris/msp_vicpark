from scipy.io import loadmat
import numpy as np

class MatlabImporter():
    """
    import the Victoria Park dataset into python

    data formats:
    * dr: dead reckoning as an array of [time, speed, steering]
    * gps: an array of [timeGps, latitude, longitude]
    * landmarks: an array of [x,y] pairs for every timestep
    """
    def __init__(self):
        """
        import data from matlab
        """

        # dead reckoning
        import_dr = loadmat('aa3_dr.mat')
        # self.dr = list(zip(import_dr['time'], import_dr['speed'], import_dr['steering']))
        self.dr = np.column_stack((import_dr['time'], import_dr['speed'], import_dr['steering']))

        # gps
        import_gps = loadmat('aa3_gpsx.mat')

        time = import_gps['timeGps']
        lat = import_gps['La_m']
        lon = import_gps['Lo_m']

        # self.gps = list(zip(time, lon, lat))
        self.gps = np.column_stack((time, lon, lat))

        # landmarks
        # pull out list of landmarks by timestep
        #   import_landmarks['data'][0][t] is each timestep
        #   import_landmarks['data'][0][t][x] and import_landmarks['data'][0][t][y] 
        #   are the arrays of coordinates for landmarks at each timestep

        # landmarks is the an array of [x,y] pairs for every timestep
        import_landmarks = loadmat('export_landmarks.mat')
        self.landmarks = [ list(zip(ts[0], ts[1])) for ts in import_landmarks['data'][0]]

        # counters for data stream
        self.dr_count  = 0
        self.gps_count = 0
        self.lm_count  = 0

    def next_gps(self):
        """
        return the next gps output
        if there is no more data, return None
        @return [timeGps, latitude, longitude]
        """
        self.gps_count += 1
        if self.gps_count <= len(self.gps):
            # return self.gps[self.gps_count-1]
            return self.gps[self.gps_count-1]
        else:
            return None

    def next_dr(self):
        """
        return the next dr output
        if there is no more data, return None
        @return [time, speed, steering]
        """
        self.dr_count += 1
        if self.dr_count <= len(self.dr):
            return self.dr[self.dr_count-1]
        else:
            return None

    def next_lm(self):
        """
        return the next landmark output
        if there is no more data, return None
        @return [ [x1,y1], [x2,y2], ... ]
        """
        self.lm_count += 1
        if self.lm_count <= len(self.landmarks):
            return self.landmarks[self.lm_count-1]
        else:
            return None

    def set_stream_count(self, **counts):
        """
        set the stream counter for the specified counter variable (ie change
        your index in the data stream).
        raises NameError for invalid counter name
        raises ValueError for non-integer count
        valid names: dr, gps, lms
        """
        for key,idx in counts.items():
            if type(idx) != int:
                raise ValueError("index must be int")
            if key == 'dr':
                self.dr_count = idx
            elif key == 'gps':
                self.gps_count = idx
            elif key == 'lms':
                self.lm_count = idx
            else:
                raise NameError("invalid counter name {}".format(key))
    
    def reset_stream_count(self, *args):
        """
        reset the stream counter for the given counter variables to 0
        (ie, go back to the start of the stream)
        raises NameError for invalid counter name
        valid names: dr, gps, lms
        """
        for name in args:
            if name == 'dr':
                set_stream_count(dr=0)
            elif name == 'gps':
                set_stream_count(gps=0)
            elif name == 'lms':
                set_stream_count(lms=0)
            else:
                raise NameError("invalid counter name {}".format(name))

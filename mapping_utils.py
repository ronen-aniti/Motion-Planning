from typing import Tuple
import csv

def read_global_home(filename: str) -> Tuple[float, float, float]:
    """
    Read in global home from CSV 2.5d map
    """
    with open(filename) as f:
        reader = csv.reader(f) 
        line_1 = next(reader)
    lat0 = float(line_1[0].split('lat0 ')[1])
    lon0 = float(line_1[1].split(' lon0 ')[1])

    return (lon0, lat0, 0.0)
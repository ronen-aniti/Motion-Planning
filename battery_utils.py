import numpy as np

def battery_consumption_rate(velocity: np.ndarray) -> float:
    # The battery consumption rate is a linear function of velocity, % per km
    base_rate = 1.0 
    consumption_factor = 2.0 
    rate = base_rate + consumption_factor * np.linalg.norm(velocity)
    return rate 
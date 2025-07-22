def predict_apogee_with_flaps(h0, v0, 
                              m=0.35, 
                              Cd_flap=1.2, 
                              A_flap=0.01274, 
                              rho=1.225, 
                              g=9.81, 
                              dt=0.001):
    """
    Predicts apogee altitude if flaps deploy immediately.
    
    Inputs:
      h0        Starting altitude (m)
      v0        Starting upward velocity (m/s)
      m         Rocket mass after burnout (kg)
      Cd_flap   Drag coefficient with deployed flaps
      A_flap    Flap frontal area (m^2)
      rho       Air density (kg/m^3)
      g         Gravitational accel (m/s^2)
      dt        Time step for Euler integration (s)
    
    Returns:
      Predicted apogee altitude (m)
    """
    v_sim = v0
    h_sim = h0
    t_sim = 0.0

    # Integrate until vertical speed drops to zero
    while v_sim > 0:
        F_d = 0.5 * rho * Cd_flap * A_flap * v_sim**2
        a   = -g - F_d/m
        v_new = v_sim + a * dt
        h_sim += 0.5 * (v_sim + v_new) * dt
        v_sim  = v_new
        t_sim += dt

    return h_sim, t_sim  # Return both altitude and time to apogee

if __name__ == "__main__":
    # **Example call** – change these to your test values:
    start_h = 190   # meters
    start_v = 46.68    # m/s upward
    apogee, time_to_apogee = predict_apogee_with_flaps(start_h, start_v)
    print(f"Predicted apogee with big flaps: {apogee:.1f} m")
    print(f"Time to apogee: {time_to_apogee:.1f} s")

    # apogee = predict_apogee_with_flaps(start_h, start_v)
    # print(f"Predicted apogee with big flaps: {apogee:.1f} m")
    # print(f"time to apogee: {apogee:.1f} m")

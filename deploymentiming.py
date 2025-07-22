def predict_apogee_with_flaps(h0, v0, 
                              m=0.35, 
                              Cd_flap=1.3, 
                              A_flap=0.01, 
                              rho=1.225, 
                              g=9.81, 
                              dt=0.1):
    """
    Predicts apogee altitude if flaps deploy immediately.
    (Your existing function)
    """
    v_sim = v0
    h_sim = h0
    # Integrate until vertical speed drops to zero
    while v_sim > 0:
        F_d = 0.5 * rho * Cd_flap * A_flap * v_sim**2
        a   = -g - F_d/m
        v_new = v_sim + a * dt
        h_sim += 0.5 * (v_sim + v_new) * dt
        v_sim  = v_new
    return h_sim


"""STILL NEEDS LOTS OF WORK DOESN'T ACCOUNT FOR AIR REISSTANCE DURING COASTING"""


def predict_apogee_no_flaps(h0, v0, g=9.81):
    """
    Predicts apogee without flaps (just gravity)
    """
    return h0 + v0**2 / (2 * g)


def find_deployment_height(v_burnout, h_burnout, target_altitude,
                          m=0.35, 
                          Cd_flap=1.3, 
                          A_flap=0.01, 
                          rho=1.225, 
                          g=9.81, 
                          dt=0.1):
    """
    Finds the height at which to deploy flaps to hit target altitude.
    
    Inputs:
      v_burnout      Velocity at motor burnout (m/s)
      h_burnout      Altitude at motor burnout (m)
      target_altitude Target apogee altitude (m)
      [other params same as predict_apogee_with_flaps]
    
    Returns:
      deployment_height  Height to deploy flaps (m), or None if impossible
    """
    
    # First check if target is achievable
    max_possible = predict_apogee_no_flaps(h_burnout, v_burnout, g)
    if target_altitude > max_possible:
        print(f"Target {target_altitude}m impossible! Max without flaps: {max_possible:.1f}m")
        return None
    
    # If flaps deployed immediately still go too high, impossible
    min_possible = predict_apogee_with_flaps(h_burnout, v_burnout, m, Cd_flap, A_flap, rho, g, dt)
    if target_altitude < min_possible:
        print(f"Target {target_altitude}m too low! Min with flaps: {min_possible:.1f}m")
        return None
    
    # Binary search to find deployment height
    low_height = h_burnout
    high_height = target_altitude  # Won't deploy above target
    
    tolerance = 1.0  # Within 1 meter is good enough
    max_iterations = 50
    
    for i in range(max_iterations):
        # Try deploying at middle height
        test_height = (low_height + high_height) / 2
        
        # Simulate coasting to test_height with no flaps
        v_at_test = (v_burnout**2 - 2*g*(test_height - h_burnout))**0.5
        
        # If velocity would be imaginary, we've gone too high
        if v_burnout**2 < 2*g*(test_height - h_burnout):
            high_height = test_height
            continue
            
        # Simulate from test height with flaps deployed
        predicted_apogee = predict_apogee_with_flaps(test_height, v_at_test, m, Cd_flap, A_flap, rho, g, dt)
        
        # Check if we're close enough
        if abs(predicted_apogee - target_altitude) < tolerance:
            return test_height
            
        # Adjust search range
        if predicted_apogee > target_altitude:
            # Deployed too late, need to deploy earlier (lower height)
            high_height = test_height
        else:
            # Deployed too early, need to deploy later (higher height)  
            low_height = test_height
    
    # If we get here, return best guess
    return (low_height + high_height) / 2


# Example usage and test function
def test_deployment_calculator():
    """Test the deployment calculator with example values"""
    
    # Example scenario
    v_burnout = 60  # m/s velocity at burnout
    h_burnout = 50  # m altitude at burnout  
    target_altitude = 200  # m target apogee
    
    print("=== Flap Deployment Calculator Test ===")
    print(f"Burnout: {h_burnout}m altitude, {v_burnout}m/s velocity")
    print(f"Target apogee: {target_altitude}m")
    print()
    
    # Find deployment height
    deploy_height = find_deployment_height(v_burnout, h_burnout, target_altitude)
    
    if deploy_height is not None:
        print(f"Deploy flaps at: {deploy_height:.1f}m")
        
        # Verify the result
        v_at_deploy = (v_burnout**2 - 2*9.81*(deploy_height - h_burnout))**0.5
        final_altitude = predict_apogee_with_flaps(deploy_height, v_at_deploy)
        
        print(f"Verification:")
        print(f"  Velocity at deployment: {v_at_deploy:.1f} m/s")
        print(f"  Predicted final altitude: {final_altitude:.1f}m")
        print(f"  Error: {abs(final_altitude - target_altitude):.1f}m")
    else:
        print("Could not find valid deployment height!")

# Run the test
if __name__ == "__main__":
    test_deployment_calculator()
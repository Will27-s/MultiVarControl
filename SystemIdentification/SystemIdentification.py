# Takes in output motor speed in counts/s
#motor_speed =  pos - pos_prev/delta_time
# G_P_dot(s) = (lambda/T)/(s_1/T)

# lambda = max(motor_speed)/max(input)
# T = the time at which this is true = 0.632 * steady_state_value

    #ifndef FUNCTIONDEF_H
    #define FUNCTIONDEF_H

    // Controllers
    float Bang_Bang_Control(int e, int u_amplitude);
    float Proportional_Control(int e, float kp);
    float Proportional_Control(int e, float kp);
    float PID_Control(int e, int e_sum, int e_prev , float kp, float ki, float kd);

    float conditional_integration(int e, int e_sum, int e_windup_limit);
    float set_u_to_max_if_out_of_bounds(float u);

    // Reference Signals
    int rotate_every_time_per_rotation_ref_signal();

    // Input Signals
    float Step_Input(int step_time);

    // Interrupt service routines 
    void readEncoderMot1();
    void readEncoderMot2();

    // dT regulator
    void dt_regulator(long delta_time_micros);

    // Data Recording
    void record_pos_data();
    void serial_plotting();

    #endif  // FUNCTIONDEF_H
    
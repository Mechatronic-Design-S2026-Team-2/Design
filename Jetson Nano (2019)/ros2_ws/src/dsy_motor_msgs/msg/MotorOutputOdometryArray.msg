uint32 last_cmd_seq
uint64 esp_cmd_rx_time_us
uint64 esp_apply_complete_time_us
# Compatibility timing field. With sweep-aware firmware this is the latest
# completed encoder sweep time, not merely the last single-drive read time.
uint64 esp_poll_complete_time_us
uint64 esp_publish_time_us

# Compatibility payload. Latest ESP firmware still places raw P18.32/P18.33
# single-turn encoder counts here so older readers remain source-compatible.
float32[6] output_phase_rad
float32 battery_voltage_v

# Sweep-aware encoder metadata. Wire order remains [rb, rm, rf, lf, lm, lb].
uint32 sweep_seq
uint8 sweep_complete
uint8 updated_mask
uint8 ok_mask
uint64 sweep_start_time_us
uint64 sweep_complete_time_us
uint64[6] encoder_sample_time_us
uint32[6] encoder_sample_seq
uint32[6] raw_encoder_single_turn_counts
int64[6] unwrapped_motor_counts
int16[6] applied_motor_rpm

# CMakeLists.txt (module root)
if ((NOT CONFIG_ZMK_SPLIT) OR CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
  target_sources(
    app PRIVATE src/value_store.c
    src/behavior_sensor_value_adjust.c
    src/behavior_mouse_move_value.c
    src/behavior_sensor_scroll_velocity.c
    src/behavior_sensor_scroll_pulse.c
    src/behavior_input_one_shot_two_axis.c
  )
  zephyr_include_directories(include)
endif()

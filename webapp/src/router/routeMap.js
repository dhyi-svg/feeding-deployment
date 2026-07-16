const routeMap = {
  home: {
    jump: '/home'
  },
  task_selection: {
    jump: '/task_selection'
  },
  robot_executing: {
    jump: '/robot_executing'
  },
  prepare_bite: {
    completed: '/bite_selection'
  },
  bite_selection: {
    jump: '/bite_selection'
  },
  bite_confirm_transfer: {
    jump: '/bite_confirm_transfer'
  },
  after_bite: {
    jump: '/after_bite'
  },
  drink_confirm_transfer: {
    jump: '/drink_confirm_transfer'
  },
  after_drink: {
    jump: '/after_drink'
  },
  wipe_confirm_transfer: {
    jump: '/wipe_confirm_transfer'
  },
  plate_release_confirm: {
    microwave: '/plate_release_confirm?location=microwave',
    table: '/plate_release_confirm?location=table',
    sink: '/plate_release_confirm?location=sink',
    jump: '/plate_release_confirm'
  },
  detection_confirm: {
    jump: '/detection_confirm'
  },
  detect_confirmation: {
    jump: '/detection_confirm'
  },
  color_correction: {
    jump: '/color_correction'
  },
  transparency: {
    jump: '/transparency'
  },
  adaptability: {
    jump: '/adaptability'
  },
  preference_correction: {
    jump: '/preference_correction'
  },
  preference_context: {
    jump: '/preference_context'
  },
  gesture_menu: {
    jump: '/gesture_menu'
  },
  gesture_setup: {
    jump: '/gesture_setup'
  },
  gesture_record_positive: {
    jump: '/gesture_record_positive'
  },
  gesture_record_negative: {
    jump: '/gesture_record_negative'
  },
  gesture_test: {
    jump: '/gesture_test'
  },
teleop: {
    jump: '/manipulation_teleop'
  },
  navigation_teleop: {
    jump: '/navigation_teleop',
    recover: '/navigation_teleop?recover=1'
  },
  nav_adjust: {
    jump: '/nav_adjust_confirm'
  },
  gesture_record: {
    jump: '/gesture_record_positive'
  },
  survey: {
    jump: '/survey'
  },
  thank_you: {
    jump: '/thank_you'
  }
};

export default routeMap;

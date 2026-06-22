export const SKILL_CATEGORY = {
  navigate_to_table: 'navigation',
  navigate_to_sink: 'navigation',
  navigate_to_fridge: 'navigation',
  navigate_to_microwave: 'navigation',

  acquire_bite: 'manipulation',
  transfer_utensil: 'manipulation',
  transfer_drink: 'manipulation',
  transfer_wipe: 'manipulation',
  emulate_transfer: 'manipulation',
  gaze_at_table: 'manipulation',
  pick_utensil: 'manipulation',
  stow_utensil: 'manipulation',
  pick_drink: 'manipulation',
  stow_drink: 'manipulation',
  pick_wipe: 'manipulation',
  stow_wipe: 'manipulation',
  open_fridge: 'manipulation',
  close_fridge: 'manipulation',
  open_microwave: 'manipulation',
  close_microwave: 'manipulation',
  press_microwave_button: 'manipulation',
  pick_plate_from_fridge: 'manipulation',
  pick_plate_from_microwave: 'manipulation',
  pick_plate_from_holder: 'manipulation',
  pick_plate_from_table: 'manipulation',
  place_plate_on_holder: 'manipulation',
  place_plate_on_table: 'manipulation',
  place_plate_in_fridge: 'manipulation',
  place_plate_in_microwave: 'manipulation',
  place_plate_in_sink: 'manipulation'
}

export function categoryOf (skillName) {
  if (!skillName) return 'manipulation'
  if (SKILL_CATEGORY[skillName]) return SKILL_CATEGORY[skillName]
  if (skillName.startsWith('navigate')) return 'navigation'
  return 'manipulation'
}

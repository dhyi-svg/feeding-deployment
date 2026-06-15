// Human-readable labels for robot skills (snake_case behavior-tree names ->
// display names). Shared by the explanation page and the teleop screens so the
// displayed skill name stays consistent across the app. Names mirror the
// behavior trees in src/feeding_deployment/actions/behavior_trees/.

export const SKILL_LABELS = {
  navigate_to_table: 'Drive to Table',
  navigate_to_fridge: 'Drive to Fridge',
  navigate_to_microwave: 'Drive to Microwave',
  navigate_to_sink: 'Drive to Sink',
  open_fridge: 'Open Fridge',
  close_fridge: 'Close Fridge',
  open_microwave: 'Open Microwave',
  close_microwave: 'Close Microwave',
  press_microwave_button: 'Start Microwave',
  pick_plate_from_fridge: 'Take Plate from Fridge',
  pick_plate_from_microwave: 'Take Plate from Microwave',
  pick_plate_from_table: 'Take Plate from Table',
  pick_plate_from_holder: 'Take Plate from Holder',
  place_plate_in_fridge: 'Put Plate in Fridge',
  place_plate_in_microwave: 'Put Plate in Microwave',
  place_plate_in_sink: 'Put Plate in Sink',
  place_plate_on_table: 'Put Plate on Table',
  place_plate_on_holder: 'Put Plate on Holder',
  gaze_at_table: 'Look at Table',
  emulate_transfer: 'Gesture Transfer',
  pick_utensil: 'Pick Up Utensil',
  acquire_bite: 'Acquire Bite',
  transfer_utensil: 'Transfer to Mouth',
  stow_utensil: 'Stow Utensil',
  pick_drink: 'Pick Up Drink',
  transfer_drink: 'Transfer Drink',
  stow_drink: 'Stow Drink',
  pick_wipe: 'Pick Up Wipe',
  transfer_wipe: 'Transfer Wipe',
  stow_wipe: 'Stow Wipe'
}

// Returns a friendly label for a skill, falling back to a title-cased version
// of the raw name (e.g. "wash_plate" -> "Wash Plate") for anything unlisted.
export function skillLabel (name) {
  if (!name) {
    return ''
  }
  if (SKILL_LABELS[name]) {
    return SKILL_LABELS[name]
  }
  return name
    .split('_')
    .map(word => word.charAt(0).toUpperCase() + word.slice(1))
    .join(' ')
}

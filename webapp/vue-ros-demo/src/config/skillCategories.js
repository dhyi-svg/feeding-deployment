// Maps each skill (the behavior-tree name published on /SkillPlan) to the kind
// of teleop the context-aware takeover button should switch to:
//   'navigation'   -> drive the mobile base (/navigation_teleop)
//   'manipulation' -> move the arm       (/manipulation_teleop)
//
// Only navigation skills strictly need an entry; anything not listed defaults to
// 'manipulation' (see categoryOf). The full list is kept here for clarity and so
// a reviewer can see every skill's intended routing at a glance.

export const SKILL_CATEGORY = {
  // --- navigation (drive the base) ---
  navigate_to_table: 'navigation',
  navigate_to_sink: 'navigation',
  navigate_to_fridge: 'navigation',
  navigate_to_microwave: 'navigation',

  // --- manipulation (move the arm) ---
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

// Resolve a skill name to a category. Unknown skills default to 'manipulation';
// the navigate_* prefix is a safety net so a newly-added navigation skill never
// mis-routes to the arm before someone adds it to the map above.
export function categoryOf (skillName) {
  if (!skillName) return 'manipulation'
  if (SKILL_CATEGORY[skillName]) return SKILL_CATEGORY[skillName]
  if (skillName.startsWith('navigate')) return 'navigation'
  return 'manipulation'
}

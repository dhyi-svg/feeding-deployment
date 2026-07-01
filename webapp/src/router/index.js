import { createRouter, createWebHashHistory } from 'vue-router'

const routes = [
  {
    path: '/',
    redirect: '/home'
  },
  {
    path: '/home',
    name: 'home',
    component: () => import('../views/home.vue')
  },
  {
    path: '/detection_confirm',
    name: 'detection_confirm',
    component: () => import('../views/detection_confirm.vue')
  },
  {
    path: '/bite_selection',
    name: 'bite_selection',
    component: () => import('../views/bite_selection.vue')
  },
{
    path: '/bite_confirm_transfer',
    name: 'bite_confirm_transfer',
    component: () => import('../views/bite_confirm_transfer.vue')
  },
  {
    path: '/after_bite',
    name: 'after_bite',
    component: () => import('../views/after_bite.vue')
  },
  {
    path: '/after_drink',
    name: 'after_drink',
    component: () => import('../views/after_drink.vue')
  },
  {
    path: '/task_selection',
    name: 'task_selection',
    component: () => import('../views/task_selection.vue')
  },
  {
    path: '/personalization',
    name: 'personalization',
    component: () => import('../views/personalization.vue')
  },
  {
    path: '/drink_confirm_transfer',
    name: 'drink_confirm_transfer',
    component: () => import('../views/drink_confirm_transfer.vue')
  },
  {
    path: '/wipe_confirm_transfer',
    name: 'wipe_confirm_transfer',
    component: () => import('../views/wipe_confirm_transfer.vue')
  },
  {
    path: '/gesture_test',
    name: 'gesture_test',
    component: () => import('../views/gesture_test.vue')
  },
  {
    path: '/robot_executing',
    name: 'robot_executing',
    component: () => import('../views/robot_executing.vue')
  },
  {
    path: '/gesture_setup',
    name: 'gesture_setup',
    component: () => import('../views/gesture_setup.vue')
  },
  {
    path: '/gesture_record_positive',
    name: 'gesture_record_positive',
    component: () => import('../views/gesture_record_positive.vue')
  },
  {
    path: '/gesture_record_negative',
    name: 'gesture_record_negative',
    component: () => import('../views/gesture_record_negative.vue')
  },
  {
    path: '/gesture_menu',
    name: 'gesture_menu',
    component: () => import('../views/gesture_menu.vue')
  },
  {
    path: '/transparency',
    name: 'transparency',
    component: () => import('../views/transparency.vue')
  },
  {
    path: '/adaptability',
    name: 'adaptability',
    component: () => import('../views/adaptability.vue')
  },
  {
    path: '/preference_correction',
    name: 'preference_correction',
    component: () => import('../views/preference_correction.vue')
  },
  {
    path: '/preference_context',
    name: 'preference_context',
    component: () => import('../views/preference_context.vue')
  },
  {
    path: '/manipulation_teleop',
    name: 'manipulation_teleop',
    component: () => import('../views/manipulation_teleop.vue')
  },
  {
    path: '/manipulation_done',
    name: 'manipulation_done',
    component: () => import('../views/manipulation_done.vue')
  },
  {
    path: '/navigation_teleop',
    name: 'navigation_teleop',
    component: () => import('../views/navigation_teleop.vue')
  },
  {
    path: '/color_correction',
    name: 'color_correction',
    component: () => import('../views/color_correction.vue')
  },
  {
    path: '/mictest',
    name: 'mictest',
    component: () => import('../views/mic_test.vue')
  },
  {
    path: '/idle_takeover',
    name: 'idle_takeover',
    component: () => import('../views/idle_takeover.vue')
  }
]

const router = createRouter({
  history: createWebHashHistory(),
  routes
})
export default router

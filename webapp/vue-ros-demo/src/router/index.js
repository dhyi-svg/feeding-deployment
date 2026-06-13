// import { createRouter, createWebHistory } from 'vue-router'
import { createRouter, createWebHashHistory } from 'vue-router'
import AboutView from '../components/OldTopBar(NoUsed).vue'

const routes = [
  {
    path: '/handleconfirmation',
    name: 'handleconfirmation',
    component: () => import(/* webpackChunkName: "about" */ '../views/handle_confirmation.vue')
  },
  {
    path: '/acquirebite',
    name: 'acquirebite',
    component: () => import(/* webpackChunkName: "about" */ '../views/acquire_bite.vue')
  },
  {
    path: '/newmealpage',
    name: 'newmealpage',
    component: () => import(/* webpackChunkName: "about" */ '../views/NewMealPage.vue')
  },
  {
    path: '/pickingup',
    name: 'pickingup',
    component: () => import(/* webpackChunkName: "about" */ '../views/pickingup.vue')
  },
  {
    path: '/notify',
    name: 'notify',
    component: () => import(/* webpackChunkName: "about" */ '../views/notify-caregiver.vue')
  },
  {
    path: '/physical',
    name: 'physical',
    component: () => import(/* webpackChunkName: "about" */ '../views/physical-button.vue')
  },
  {
    path: '/transfermeal',
    name: 'transfermeal',
    component: () => import(/* webpackChunkName: "about" */ '../views/transfer_meal.vue')
  },
  {
    path: '/executingbitetransfer',
    name: 'executingbitetransfer',
    component: () => import(/* webpackChunkName: "about" */ '../views/Executing_Bite_Transfer.vue')
  },
  {
    path: '/afterbitetransfer',
    name: 'afterbitetransfer',
    component: () => import(/* webpackChunkName: "about" */ '../views/after_bite_transfer.vue')
  },
  {
    path: '/test',
    name: 'test',
    component: () => import(/* webpackChunkName: "about" */ '../views/Test.vue')
  },
  {
    path: '/afterdrinktransfer',
    name: 'afterdrinktransfer',
    component: () => import(/* webpackChunkName: "about" */ '../views/after_drink_transfer.vue')
  },
  {
    path: '/task_selection',
    name: 'task_selection',
    component: () => import(/* webpackChunkName: "about" */ '../views/task_selection.vue')
  },
  {
    path: '/swithtodrink',
    name: 'swithtodrink',
    component: () => import(/* webpackChunkName: "about" */ '../views/swith_to_drink.vue')
  },
  {
    path: '/transferdrinks',
    name: 'transferdrinks',
    component: () => import(/* webpackChunkName: "about" */ '../views/transferdrinks.vue')
  },
  {
    path: '/executingdrinktransfer',
    name: 'executingdrinktransfer',
    component: () => import(/* webpackChunkName: "about" */ '../views/Executing_Drink_Transfer.vue')
  },
  {
    path: '/wipingtrans',
    name: '/wipingtrans',
    component: () => import(/* webpackChunkName: "about" */ '../views/wipingtrans.vue')
  },
  {
    path: '/wiping',
    name: 'wiping',
    component: () => import(/* webpackChunkName: "about" */ '../views/wiping.vue')
  },
  {
    path: '/wipingprocess',
    name: 'wipingprocess',
    component: () => import(/* webpackChunkName: "about" */ '../views/wipingprocess.vue')
  },
  {
    path: '/callbeforetransfer',
    name: 'callbeforetransfer',
    component: () => import(/* webpackChunkName: "about" */ '../views/call-before-transfer.vue')
  },
  {
    path: '/home',
    name: 'home',
    component: () => import(/* webpackChunkName: "about" */ '../views/home.vue')
  },
  {
    path: '/CustomTextDisplay',
    name: 'CustomTextDisplay',
    component: () => import(/* webpackChunkName: "about" */ '../views/CustomTextDisplay.vue')
  },
  {
    path: '/gesturetest',
    name: 'Gesturetest',
    component: () => import(/* webpackChunkName: "about" */ '../views/gesturetest.vue')
  },
  {
    path: '/gesturemoveback',
    name: 'Gesturemoveback',
    component: () => import(/* webpackChunkName: "about" */ '../views/gesturemoveback.vue')
  },
  {
    path: '/preparepickup',
    name: 'preparepickup',
    component: () => import(/* webpackChunkName: "about" */ '../views/PrepareBitePickup.vue')
  },
  {
    path: '/gesturemove',
    name: 'GestureMove',
    component: () => import(/* webpackChunkName: "about" */ '../views/gesturemove.vue')
  },
  {
    path: '/gesturesetting',
    name: 'Gesturesetting',
    component: () => import(/* webpackChunkName: "about" */ '../views/GestureSetting.vue')
  },
  {
    path: '/gesturerecording',
    name: 'GestureRecording',
    component: () => import(/* webpackChunkName: "about" */ '../views/GestureRecording.vue')
  },
  {
    path: '/gesturerecording2',
    name: 'GestureRecording2',
    component: () => import(/* webpackChunkName: "about" */ '../views/GestureRecording2.vue')
  },
  {
    path: '/gesturemain',
    name: 'gesturemain',
    component: () => import(/* webpackChunkName: "about" */ '../views/GestureMain.vue')
  },
  {
    path: '/robotbehavior',
    name: 'robotbehavior',
    component: () => import(/* webpackChunkName: "about" */ '../views/Robotbehavior.vue')
  },
  {
    path: '/fixedconfigurations',
    name: 'fixedconfigurations',
    component: () => import(/* webpackChunkName: "about" */ '../views/fixedconfigurations.vue')
  },
  {
    path: '/preference_correction',
    name: 'preference_correction',
    component: () => import(/* webpackChunkName: "about" */ '../views/PreferenceCorrection.vue')
  },
  {
    path: '/preference_context',
    name: 'preference_context',
    component: () => import(/* webpackChunkName: "about" */ '../views/PreferenceContext.vue')
  },
  {
    path: '/gesturemove2',
    name: 'gesturemove2',
    component: () => import(/* webpackChunkName: "about" */ '../views/gesturemove2.vue')
  },
  {
    path: '/preparepickup2',
    name: 'preparepickup2',
    component: () => import(/* webpackChunkName: "about" */ '../views/PrepareBitePickup2.vue')
  },
  {
    path: '/teleop',
    name: 'teleop',
    component: () => import(/* webpackChunkName: "about" */ '../views/Teleop.vue')
  },
  {
    path: '/resuming',
    name: 'resuming',
    component: () => import(/* webpackChunkName: "about" */ '../views/Resuming.vue')
  }
]

const router = createRouter({
  history: createWebHashHistory(),
  // history: createWebHistory(process.env.BASE_URL),
  routes
})
export default router

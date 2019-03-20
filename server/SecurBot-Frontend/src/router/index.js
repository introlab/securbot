import Vue from 'vue'
import Router from 'vue-router'

import Layout from '@/components/Layout.vue'
import Teleop from '@/components/pages/Teleop.vue'
import Patrol from '@/components/pages/Patrol.vue'
import Events from '@/components/pages/Events.vue'

Vue.use(Router)

/*
* Author(s) : Edouard Legare <edouard.legare@usherbrooke.ca>
* File : index.js
* Desc : Routing file use by the router to show desired path.
*
>>Current routing<<
The routing only start at the router link

>index.html
  │
  └─>APP.vue
      │
      └─>Layout
          │
          ├─>Navigation Bar
          │
          └─>Router-Link
              │
              ├─>Teleoperation Page
              │
              ├─>Patrol Page
              │
              └─>Events Page
*/

export default new Router({
  routes: [
    {
      path: '/',
      component: Layout,
      redirect: {
        name: "SecurBot Teleoperation"
      },
      children:[
        {
          path: "teleop",
          name: "SecurBot Teleoperation",
          component: Teleop,
          props: true
        },
        {
          path: "patrol",
          name: "SecurBot Patrol Planner",
          component: Patrol,
          props: true
        },
        {
          path: "logs",
          name: "SecurBot Event Logging",
          component: Events,
          props: true
        },        
      ]
    }
  ]
})

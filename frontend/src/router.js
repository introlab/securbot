/**
 * Routing file used by the router to show desired path.
 * @module router
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';
import Router from 'vue-router';

import Layout from '@/components/Layout';
import Teleop from '@/components/pages/Teleop';
import Patrol from '@/components/pages/Patrol';
import Events from '@/components/pages/Events';
import Testing from '@/components/pages/Testing';
import Testing2 from '@/components/pages/Testing2';

Vue.use(Router);

/*
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
      redirect: {
        name: 'SecurBot Teleoperation',
      },
    },
    {
      path: '/operator',
      component: Layout,
      redirect: {
        name: 'SecurBot Teleoperation',
      },
      children: [
        {
          path: 'teleop',
          name: 'SecurBot Teleoperation',
          component: Teleop,
          props: true,
        },
        {
          path: 'patrol',
          name: 'SecurBot Patrol Planner',
          component: Patrol,
          props: true,
        },
        {
          path: 'logs',
          name: 'SecurBot Event Logging',
          component: Events,
          props: true,
        },
        {
          path: 'testing',
          component: Testing,
          props: false,
        },
      ],
    },
    {
      path: '/testing2',
      component: Testing2,
    },
  ],
});

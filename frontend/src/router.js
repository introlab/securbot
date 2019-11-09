/**
 * Routing file used by the router to show desired path.
 * @module router
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';
import Router from 'vue-router';

import Layout from '@/components/Layout';
import Teleop from '@/components/views/Teleop';
import Patrol from '@/components/views/Patrol';
import Events from '@/components/views/Events';
import Testing from '@/components/views/Testing';
import Testing2 from '@/components/views/Testing2';
import NextTeleop from '@/components/views/TeleopNext';

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
        },
        {
          path: 'patrol',
          name: 'SecurBot Patrol Planner',
          component: Patrol,
        },
        {
          path: 'logs',
          name: 'SecurBot Event Logging',
          component: Events,
        },
        {
          path: 'testing',
          component: Testing,
        },
        {
          path: 'next',
          component: NextTeleop,
        },
      ],
    },
    {
      path: '/testing2',
      component: Testing2,
    },
  ],
});

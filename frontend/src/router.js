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
import PatrolPlanner from '@/components/views/PatrolPlanner';
import PatrolConfig from '@/components/views/PatrolConfig';
import Events from '@/components/views/Events';
// import Testing from '@/components/views/Testing';
import Testing2 from '@/components/views/Testing2';

import Mega from '@/components/views/MegaGenial';

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
          path: 'patrol-planner',
          name: 'SecurBot Patrol Planner',
          component: PatrolPlanner,
        },
        {
          path: 'patrol-config',
          name: 'SecurBot Patrol Config',
          component: PatrolConfig,
        },
        {
          path: 'logs',
          name: 'SecurBot Event Logging',
          component: Events,
        },
        {
          path: 'mega-genial',
          name: 'Securbot MegaGenial',
          component: Mega,
        },
      ],
    },
    {
      path: '/testing2',
      component: Testing2,
    },
  ],
});

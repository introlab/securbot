/**
 * Routing config files. Includes all the routes that the app can send a user to.
 *
 * @module router
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.1.0
 */

import Vue from 'vue';
import Router from 'vue-router';

import Layout from '@/components/Layout';
import Teleop from '@/components/views/Teleop';
import PatrolPlanner from '@/components/views/PatrolPlanner';
import PatrolConfig from '@/components/views/PatrolConfig';
import Events from '@/components/views/Events';
import TestingRobot from '@/components/views/TestingRobot';

import Mega from '@/components/views/MegaGenial';

Vue.use(Router);

/*
>index.html
  │
  └─>APP.vue - The App parent component
      │
      └─>Layout - A component that sets the view sizing and includes the navbar
      |    │
      |    ├─>Teleoperation View - The view used for manually command the robot and see its feeds
      |    │
      |    ├─>Patrol Planner View - The view used to plan a patrol, shows the map and waypoint list
      │    |
      |    ├─>Patrol Config View - The view used to config patrols and schedules
      |    │
      |    ├─>Events View - The view used to visualize events saved in the database
      |    |
      |    └─>MegaGenial - A view used for demo at the MegaGenial Event, kept as example of multi-
      |                       screen view
      |
      └─>Testing Robot
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
      path: '/TestingRobot',
      component: TestingRobot,
    },
  ],
});
